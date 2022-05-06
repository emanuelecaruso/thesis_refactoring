#include "LinSystemBA.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"

bool MeasBA::getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel){
  // project active point get pixel of projected active point
  Eigen::Vector2f uv;
  cam_couple->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  cam_couple->cam_m_->uv2pixelCoords( uv, pixel, active_point->level_ );

  if(cam_couple->cam_m_->pyramid_->getC(active_point->level_)->pixelInRange(pixel))
    return true;
  return false;
}

float MeasBA::getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type){
  float z, z_hat;
  float error;
  if(image_type==INTENSITY_ID){
    z = active_point->c_;
    z_hat = cam_couple->cam_m_->pyramid_->getC(active_point->level_)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }
  else if(image_type==GRADIENT_ID){
    z = active_point->magn_cd_;
    z_hat = cam_couple->cam_m_->pyramid_->getMagn(active_point->level_)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }
  return error;

}

Eigen::Matrix<float,1,2> MeasBA::getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getCDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getCDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getMagnDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getMagnDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }

  return image_jacobian;
}

void MeasBA::loadJacobians(ActivePoint* active_point){

    J_m.setZero();  // initialize J_m
    J_r.setZero();  // initialize J_m
    J_d=0;


    // get Jm_
    Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_(active_point);
    // Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_old_(active_point);
    Eigen::Matrix<float,2,6> Jr_ = cam_couple_->getJr_(active_point);
    Eigen::Matrix<float,2,1> Jd_ = cam_couple_->getJd_(active_point);

    // update J_m and error for intensity
    Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian(pixel_, active_point, cam_couple_, INTENSITY_ID);
    J_m += image_jacobian_intensity*Jm_;
    J_r += image_jacobian_intensity*Jr_;
    J_d += image_jacobian_intensity*Jd_;

    // // update J_m and error for gradient
    // Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian(pixel_, active_point, cam_couple_, GRADIENT_ID);
    // J_m += image_jacobian_gradient*Jm_;
    // J_r += image_jacobian_gradient*Jr_;
    // J_d += image_jacobian_gradient*Jd_;

    J_m_transpose= J_m.transpose();
    J_r_transpose= J_r.transpose();
}


bool MeasBA::init(ActivePoint* active_point, CamCouple* cam_couple, float chi_occlusion_threshold){
  assert(active_point->cam_==cam_couple->cam_r_);

  // project active point and get pixel coordinates
  bool pixel_in_cam = getPixelOfProjectedActivePoint(active_point, cam_couple, pixel_);
  // if pixel is outside of frustum
  if(!pixel_in_cam){
    valid_ = false;
    return false;
  }

  error=0;  // initialize error
  error += getError( pixel_,  active_point, cam_couple, INTENSITY_ID);
  // error += getError( pixel_,  active_point, cam_couple, GRADIENT_ID);

  // control on error
  if(abs(error)>chi_occlusion_threshold){
    occlusion_ = true;
    return false;
  }

  return true;
}




void LinSysBA::reinitWithNewPoints(int n_points){
  // load number of points
  p_size = n_points;

  H_cp.resize(c_size,p_size);
  H_pp.resize(p_size);
  b_p.resize(p_size);
  dx_p.resize(p_size);

  H_cc.setZero();
  b_c.setZero();
  dx_c.setZero();
  H_cp.setZero();
  H_pp.setZero();
  b_p.setZero();
  dx_p.setZero();

  chi=0;
}

float LinSysBA::addMeasurement(MeasBA* measurement, int p_idx){
  assert(measurement->cam_couple_->cam_r_!=measurement->cam_couple_->cam_m_);

  bool no_r = measurement->cam_couple_->cam_r_->fixed_;
  bool no_m = measurement->cam_couple_->cam_m_->fixed_;

  int r_idx, m_idx;

  if(!no_r){
    assert(r_idx < c_size);
    r_idx = measurement->cam_couple_->cam_r_->cam_data_for_ba_->c_idx_*6;
  }

  if(!no_m){
    assert(m_idx < c_size);
    m_idx = measurement->cam_couple_->cam_m_->cam_data_for_ba_->c_idx_*6;
  }

  assert(p_idx < p_size);

  // get weight
  float weight = getWeight(measurement);

  // ********* H *********

  // CAM CAM BLOCK

  // m-m
  if(!no_m){
    H_cc.block<6,6>(m_idx,m_idx).triangularView<Eigen::Upper>() += measurement->J_m_transpose*weight*measurement->J_m;
  }

  // // r-r
  if(!no_r){
    H_cc.block<6,6>(r_idx,r_idx).triangularView<Eigen::Upper>() += measurement->J_r_transpose*weight*measurement->J_r;
  }

  // m-r
  if(!no_r && !no_m){
    if(m_idx<r_idx){
      H_cc.block<6,6>(m_idx,r_idx) += measurement->J_m_transpose*weight*measurement->J_r;
    }
    else{
      H_cc.block<6,6>(r_idx,m_idx) += measurement->J_r_transpose*weight*measurement->J_m;
    }
  }

  // CAM POINT BLOCK

  // m-p
  if(!no_m){
    H_cp.block<6,1>(m_idx,p_idx) += measurement->J_m_transpose*weight*measurement->J_d;
  }

  // r-p
  if(!no_r){
    H_cp.block<6,1>(r_idx,p_idx) += measurement->J_r_transpose*weight*measurement->J_d;
  }

  // POINT POINT BLOCK

  // p
  H_pp(p_idx) += measurement->J_d*weight*measurement->J_d;

  // ********* b *********

  // cam segment
  // m
  if(!no_m){
    b_c.segment<6>(m_idx) += measurement->J_m_transpose*weight*measurement->error;
  }

  // r
  if(!no_r){
    b_c.segment<6>(r_idx) += measurement->J_r_transpose*weight*measurement->error;
  }

  // p
  b_p(p_idx) += measurement->J_d*weight*measurement->error;

  return measurement->error*weight*measurement->error;
}


void LinSysBA::buildLinearSystem(std::vector<std::vector<MeasBA*>*>& measurement_vec_vec ){
  int count = 0;

  // iterate through all measurements
  for(int i=0; i<measurement_vec_vec.size(); i++){
    std::vector<MeasBA*>* v = measurement_vec_vec[i];
    for(MeasBA* measurement : *v){
      count++;
      chi += addMeasurement(measurement, i);
    }
  }

  // *********  MARG PRIORS  *********

  // build linear system marg
  // LinSysBAMarg lin_sys_marg(this);
  // lin_sys_marg.loadMargPriors();


  chi /= (float)count;

  // visualizeH();
  // iterate through all marginalized measurements
  // TODO
}

bool LinSysBA::visualizeH(){
  Image<colorRGB>* img_H = new Image<colorRGB>("H");
  int size = c_size+p_size;
  if (size == 0)
    return false;
  // img_H->initImage(c_size,c_size);
  img_H->initImage(size,size);
  img_H->setAllPixels( white);

  // pose pose block
  for(int i=0; i<c_size; i++){
    for(int j=0; j<c_size; j++){
      float val;
      if(i<j){
        val=H_cc(i,j);
      }
      else{
        val=H_cc(j,i);
      }
      if (val!=0){
        img_H->setPixel(i,j, red);
      }
      else{
        img_H->setPixel(i,j, white);
      }
    }
  }

  // pose point block
  for(int i=0; i<c_size; i++){
    for(int j=0; j<p_size; j++){
      if ((H_cp)(i,j)!=0){
        img_H->setPixel(i,c_size+j, green);
      }
      else{
        img_H->setPixel(i,c_size+j, white);

      }
    }
  }

  // point pose block
  for(int i=0; i<p_size; i++){
    for(int j=0; j<c_size; j++){
      if ((H_cp)(j,i)!=0){
        img_H->setPixel(i+c_size,j, green);
      }
      else{
        img_H->setPixel(i+c_size,j, white);

      }
    }
  }

  // point point block
  for(int i=0; i<p_size; i++){
    if (H_pp(i)!=0){
      img_H->setPixel(i+c_size,i+c_size, blue);
    }
    else{
      img_H->setPixel(i+c_size,i+c_size, white);

    }

  }

  img_H->show(1);
  waitkey(0);
  delete img_H;

}

void LinSysBA::updateCameras(){

  // iterate through keyframes
  for(int i=0; i<dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];

    if (keyframe->fixed_)
      continue;

    Vector6f dx_curr = dx_c.segment<6>(keyframe->cam_data_for_ba_->c_idx_*6);

    Eigen::Isometry3f frame_camera_wrt_world =(*(keyframe->frame_camera_wrt_world_))*v2t_inv(dx_curr);
    keyframe->assignPose( frame_camera_wrt_world );

  }



}

void LinSysBA::updatePoints(){
  // iterate through active keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[i];

    for( ActivePoint* active_pt : cam_r->points_container_->active_points_ ){
      if(active_pt->p_idx_!=-1){

        float new_invdepth = active_pt->invdepth_ + dx_p(active_pt->p_idx_);
        new_invdepth = std::min(new_invdepth,(float)1.0/cam_r->cam_parameters_->min_depth);
        new_invdepth = std::max(new_invdepth,(float)1.0/cam_r->cam_parameters_->max_depth);
        active_pt->updateInvdepthVarAndP( new_invdepth , (1.0/(H_pp[active_pt->p_idx_]+0.001))+0.001 );
      }
    }
  }
}

void LinSysBA::updateState(){

  // Eigen::VectorXf H_pp_damped = H_pp + dso_->parameters_->damp_point_invdepth * Eigen::VectorXf::Ones(H_pp.size());
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic> H_pp_inv = pinvDiagonalMatrix(H_pp_damped );
  // dx_c = H_cc.selfadjointView<Eigen::Upper>().ldlt().solve(-b_c);
  // dx_p = H_pp_inv*(-b_p);


  // get inverse of schur -> (H_cc - H_cp H_pp_inv H_pc)_inv
  Eigen::VectorXf H_pp_damped = H_pp + dso_->parameters_->damp_point_invdepth * Eigen::VectorXf::Ones(H_pp.size());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic> H_pp_inv = invDiagonalMatrix(H_pp_damped );
  H_cc = H_cc.selfadjointView<Eigen::Upper>();
  Eigen::MatrixXf H_pc = H_cp.transpose();
  Eigen::MatrixXf Schur_inv = (H_cc - H_cp * H_pp_inv * H_pc).inverse();

  dx_c = Schur_inv * ( -b_c + H_cp * H_pp_inv * b_p);
  dx_p = H_pp_inv * ( -b_p - H_pc * dx_c );

  updateCameras();
  updatePoints();

  if(dso_->parameters_->debug_optimization){

    dso_->spectator_->renderState();
    dso_->spectator_->showSpectator();
    std::cout << "chi " << chi << std::endl;
  }

}

void LinSysBA::init(){

  // count number of cameras
  int count =0;

  // iterate through keyframes
  for(int i=0; i<dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    if(!keyframe->fixed_){
      keyframe->cam_data_for_ba_->c_idx_=count;
      count++;
    }
  }

  c_size = count*6;

  H_cc.resize(c_size,c_size);
  b_c.resize(c_size);
  dx_c.resize(c_size);

}

float LinSysBA::getWeight(MeasBA* measurement){
  float weight=1;
  // float weight=1.0/measurement->active_point_->invdepth_var_;

  // huber robustifier
  if(dso_->parameters_->opt_norm==HUBER){
    float huber_threshold=dso_->parameters_->huber_threshold;
    float u = abs(measurement->error);

    if (u<=huber_threshold){
      weight*=1/huber_threshold;
    }
    else{
      // float rho_der = huberNormDerivative(error,dso_->parameters_->huber_threshold);
      float rho_der = huberNormDerivative(u,dso_->parameters_->huber_threshold);
      float gamma=(1/u)*rho_der;
      weight*=gamma;
    }

  }
  // least square without robustifier
  else if (dso_->parameters_->opt_norm==QUADRATIC){

  }
  else{
    throw std::invalid_argument( "optimization norm has wrong value" );
  }
  assert(!std::isnan(weight));
  assert(!std::isinf(weight));


  // // weight
  // float variance = dso_->parameters_->variance;
  // int ni = dso_->parameters_->robustifier_dofs;
  // float weight = (ni+1.0)/(ni+(pow(error,2)/variance));
  //
  // float  weight = weight*gamma;

  // return weight;
  return weight;

}

bool PriorMeas::init(ActivePoint* active_point, CamCouple* cam_couple){
  assert(cam_couple->cam_r_==active_point->cam_);
  // project active point
  Eigen::Vector2f uv;
  cam_couple->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  bool uv_in_range = active_point->cam_->uvInRange(uv);

  if(!uv_in_range){
    valid_=false;
    return false;
  }


  J_m.setZero();  // initialize J_m
  error=0;  // initialize error

  // get Jm_
  Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_(active_point);
  // Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_old_(active_point);


  // update J_m and error for intensity
  Eigen::Matrix<float,1,2> image_jacobian_intensity = dso->parameters_->intensity_coeff*getImageJacobian(pixel, active_point, cam_couple, level, INTENSITY_ID);
  J_m += image_jacobian_intensity*Jm_;
  error += dso->parameters_->intensity_coeff*getError( pixel,  active_point, cam_couple, level, INTENSITY_ID);


  // // update J_m and error for gradient
  // Eigen::Matrix<float,1,2> image_jacobian_gradient = dso->parameters_->gradient_coeff*dso->parameters_->gradient_coeff*getImageJacobian(pixel, active_point, cam_couple, level, GRADIENT_ID);
  // J_m += image_jacobian_gradient*Jm_;
  // error += dso->parameters_->intensity_coeff*getError( pixel,  active_point, cam_couple, level, GRADIENT_ID);

  assert(abs(error)<1);
  J_m_transpose= J_m.transpose();

  return true;

}

void LinSysBAMarg::init(){

  // iterate through keyframes
  for(int i=0; i<lin_sys_ba_->dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = lin_sys_ba_->dso_->cameras_container_->keyframes_active_[i];

    // if keyframe has priors

      // set index

      // iterate through priors


  }
}
