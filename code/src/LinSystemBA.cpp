#include "LinSystemBA.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"


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
    Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian( INTENSITY_ID);
    J_m.head<6>() += image_jacobian_intensity*Jm_;
    J_r.head<6>() += image_jacobian_intensity*Jr_;
    J_d += image_jacobian_intensity*Jd_;


    // update J_m and error for gradient
    if(image_id==GRADIENT_ID){
      Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian( GRADIENT_ID);
      J_m.head<6>() += image_jacobian_gradient*Jm_;
      J_r.head<6>() += image_jacobian_gradient*Jr_;
      J_d += image_jacobian_gradient*Jd_;
    }

    J_m_transpose= J_m.transpose();
    J_r_transpose= J_r.transpose();
}



void LinSysBA::reinitWithNewPoints(int n_points){
  // load number of points
  p_size_ = n_points;

  H_cp_.resize(c_size_,p_size_);
  H_pp_.resize(p_size_);
  b_p_.resize(p_size_);
  dx_p.resize(p_size_);
  omega_.resize(p_size_);

  clear();
  dx_p.setZero();
  dx_c.setZero();

  chi=0;
}

float LinSysBA::addMeasurement(MeasBA* measurement, int p_idx){
  assert(measurement->cam_couple_!=nullptr);

  bool no_r = measurement->cam_couple_->cam_r_->fixed_ || measurement->cam_couple_->cam_r_->marginalized_;
  bool no_m = measurement->cam_couple_->cam_m_->fixed_ || measurement->cam_couple_->cam_m_->marginalized_;

  int r_idx = measurement->cam_couple_->cam_r_->cam_data_for_ba_->c_idx_;
  int m_idx = measurement->cam_couple_->cam_m_->cam_data_for_ba_->c_idx_;

  if(!no_r){
    assert(r_idx < c_size_);
    r_idx = measurement->cam_couple_->cam_r_->cam_data_for_ba_->c_idx_*J_SZ;
  }

  if(!no_m){
    assert(m_idx < c_size_);
    m_idx = measurement->cam_couple_->cam_m_->cam_data_for_ba_->c_idx_*J_SZ;
  }

  assert(p_idx < p_size_);

  // get weight
  float weight = measurement->getWeight();
  assert(std::isfinite(weight));
  assert(measurement->J_m_transpose.allFinite());
  assert(measurement->J_m.allFinite());
  assert(std::isfinite(measurement->error));
  assert(measurement->J_r.allFinite());
  assert(measurement->J_r_transpose.allFinite());

  // ********* H *********

  // CAM CAM BLOCK

  // m-m
  if(!no_m){
    H_cc_.block<J_SZ,J_SZ>(m_idx,m_idx).triangularView<Eigen::Upper>() += measurement->J_m_transpose*weight*measurement->J_m;
  }

  // // r-r
  if(!no_r){
    H_cc_.block<J_SZ,J_SZ>(r_idx,r_idx).triangularView<Eigen::Upper>() += measurement->J_r_transpose*weight*measurement->J_r;
  }

  // m-r
  if(!no_r && !no_m){
    if(m_idx<r_idx){
      H_cc_.block<J_SZ,J_SZ>(m_idx,r_idx) += measurement->J_m_transpose*weight*measurement->J_r;
    }
    else{
      H_cc_.block<J_SZ,J_SZ>(r_idx,m_idx) += measurement->J_r_transpose*weight*measurement->J_m;
    }
  }

  // CAM POINT BLOCK

  // m-p
  if(!no_m){
    H_cp_.block<J_SZ,1>(m_idx,p_idx) += measurement->J_m_transpose*(weight*measurement->J_d);
  }

  // r-p
  if(!no_r){
    H_cp_.block<J_SZ,1>(r_idx,p_idx) += measurement->J_r_transpose*(weight*measurement->J_d);
  }

  // POINT POINT BLOCK

  // p
  H_pp_(p_idx) += measurement->J_d*weight*measurement->J_d;


  // ********* b *********

  // cam segment
  // m
  if(!no_m){
    b_c_.segment<J_SZ>(m_idx) += measurement->J_m_transpose*(weight*measurement->error);
  }

  // r
  if(!no_r){
    b_c_.segment<J_SZ>(r_idx) += measurement->J_r_transpose*(weight*measurement->error);
  }

  // p
  b_p_(p_idx) += measurement->J_d*weight*measurement->error;


  // variance

  omega_(p_idx) += measurement->J_d*(1.0/measurement->var_)*measurement->J_d;

  float chi = measurement->error*weight*measurement->error;
  assert(std::isfinite(chi));
  return chi;
}


void LinSysBA::integrateMargPriors(MarginalizationHandler* marginalization_handler_){

  // H_cc_.setZero();
  // H_cp_.setZero();
  // H_pp_.setZero();
  // b_c_.setZero();
  // b_p_.setZero();

  // iterate through keyframes with priors
  for( int i=0; i<marginalization_handler_->keyframes_with_priors_.size(); i++ ){
    CameraForMapping* cam_r = marginalization_handler_->keyframes_with_priors_[i];
    bool no_r = cam_r->fixed_ || cam_r->marginalized_;
    if(no_r)
      continue;

    int c_marg_idx_r = cam_r->cam_data_for_ba_->c_marg_idx_*J_SZ;
    int c_idx_r = cam_r->cam_data_for_ba_->c_idx_*J_SZ;
    H_cc_.block<J_SZ,J_SZ>(c_idx_r,c_idx_r).triangularView<Eigen::Upper>() += marginalization_handler_->H_tilde_.block<J_SZ,J_SZ>(c_marg_idx_r,c_marg_idx_r);
    b_c_.segment<J_SZ>(c_idx_r) += marginalization_handler_->b_tilde_.segment<J_SZ>(c_marg_idx_r);

    // off diagonal terms
    for( int j=i+1; j<marginalization_handler_->keyframes_with_priors_.size(); j++ ){
      CameraForMapping* cam_m = marginalization_handler_->keyframes_with_priors_[j];
      bool no_m = cam_m->fixed_ || cam_m->marginalized_;
      if(no_m)
        continue;

      int c_idx_m = cam_m->cam_data_for_ba_->c_idx_*J_SZ;
      int c_marg_idx_m = cam_m->cam_data_for_ba_->c_marg_idx_*J_SZ;

      assert(c_idx_r>=0 && c_idx_r<H_cc_.rows() || no_r);
      assert(c_idx_m>=0 && c_idx_m<H_cc_.rows() || no_m);
      assert(c_marg_idx_r>=0 && c_marg_idx_r<marginalization_handler_->H_tilde_.rows() );
      assert(c_marg_idx_m>=0 && c_marg_idx_m<marginalization_handler_->H_tilde_.rows() );

      // m-r
      if(!no_r && !no_m){
        if(c_idx_m<c_idx_r){
          H_cc_.block<J_SZ,J_SZ>(c_idx_m,c_idx_r) += marginalization_handler_->H_tilde_.block<J_SZ,J_SZ>(c_marg_idx_m,c_marg_idx_r);
        }
        else{
          H_cc_.block<J_SZ,J_SZ>(c_idx_r,c_idx_m) += marginalization_handler_->H_tilde_.block<J_SZ,J_SZ>(c_marg_idx_r,c_marg_idx_m);
        }
      }

    }
  }

}

void LinSysBA::buildLinearSystem(std::vector<std::vector<MeasBA*>*>& measurement_vec_vec, MarginalizationHandler* marginalization_handler ){
  int count = 0;
  assert(b_p_.allFinite());
  assert(H_cp_.allFinite());
  assert(b_c_.allFinite());

  // iterate through all measurements
  for(int i=0; i<measurement_vec_vec.size(); i++){
    std::vector<MeasBA*>* v = measurement_vec_vec[i];
    for(MeasBA* measurement : *v){
      count++;
      chi += addMeasurement(measurement, i);
    }
  }

  assert(measurement_vec_vec.size()>0);
  assert(count>0);
  assert(b_p_.allFinite());
  assert(H_cp_.allFinite());
  assert(b_c_.allFinite());

  chi /= (float)count;

  // *********  MARG PRIORS  *********


  // integrate marginalization priors
  if(do_marginalization){
    integrateMargPriors(marginalization_handler);
  }

  // // *********  EXPOSURE PRIORS  *********
  //
  // integrateExposurePriors();

  // visualize hessian / b
  // visualizeH();
  // visualizeB();
}


void LinSysBA::updateCameras(){

  // iterate through keyframes
  for(int i=0; i<dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];

    if (keyframe->fixed_)
      continue;

    Eigen::Matrix<float,J_SZ,1> dx_curr = dx_c.segment<J_SZ>(keyframe->cam_data_for_ba_->c_idx_*J_SZ);
    Vector6f dx_pose = dx_curr.head<6>();

    Eigen::Isometry3f frame_camera_wrt_world =(*(keyframe->frame_camera_wrt_world_))*v2t_inv(dx_pose);
    keyframe->assignPose( frame_camera_wrt_world );

  }



}

void LinSysBA::updatePoints(){
  // iterate through active keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->frames_with_active_pts_[i];

    for( ActivePoint* active_pt : cam_r->points_container_->active_points_ ){
      if(active_pt->p_idx_!=-1){
        if(active_pt->p_idx_>=dx_p.size()){
          std::cout << active_pt->p_idx_ << " " << dx_p.size() << " " << active_pt->cam_->name_ << " " << active_pt->cam_->marginalized_  << std::endl;

        }
        assert(active_pt->p_idx_>=0 && active_pt->p_idx_<dx_p.size());
        float new_invdepth = active_pt->invdepth_ + dx_p(active_pt->p_idx_);
        new_invdepth = std::min(new_invdepth,(float)1.0/cam_r->cam_parameters_->min_depth);
        new_invdepth = std::max(new_invdepth,(float)1.0/cam_r->cam_parameters_->max_depth);

        Eigen::VectorXf col = H_cp_.col(active_pt->p_idx_);
        int n = col.rows()-(std::count(col.data(), col.data() + col.size(), 0));
        // float var = ( 1.0/ ( (H_pp_[active_pt->p_idx_]) ) )+10e-32;
        // float var = ( 1.0/ ( (H_pp_[active_pt->p_idx_]) ) )+0.00000000001;
        float var = (1.0/( (omega_(active_pt->p_idx_)/(n/J_SZ)) +0.00001))+0.00001;
        // std::cout << var << " " << omega_(active_pt->p_idx_) << " " << n << std::endl;
        // std::cout << H_pp_[active_pt->p_idx_] << " " << var << " " << n <<  std::endl;

        active_pt->updateInvdepthVarAndP( new_invdepth , var );
      }
    }
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

  c_size_ = count*J_SZ;

  H_cc_.resize(c_size_,c_size_);
  b_c_.resize(c_size_);
  dx_c.resize(c_size_);

  clear();
}


void PriorMeas::loadJacobians(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple){
  assert(cam_couple->cam_r_==active_point->cam_);

  J_m.setZero();  // initialize J_m
  J_d=0;

  // get Jm_
  Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_(active_point);
  // Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_old_(active_point);
  Eigen::Matrix<float,2,1> Jd_ = cam_couple_->getJd_(active_point);

  // update J_m and error for intensity
  Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian( INTENSITY_ID);
  J_m.head<6>() += image_jacobian_intensity*Jm_;
  J_d += image_jacobian_intensity*Jd_;


  // update J_m and error for gradient
  if(image_id==GRADIENT_ID){
    Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian( GRADIENT_ID);
    J_m.head<6>() += image_jacobian_gradient*Jm_;
    J_d += image_jacobian_gradient*Jd_;
  }

  J_m_transpose= J_m.transpose();

}
