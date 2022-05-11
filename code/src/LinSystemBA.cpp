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
    J_m += image_jacobian_intensity*Jm_;
    J_r += image_jacobian_intensity*Jr_;
    J_d += image_jacobian_intensity*Jd_;


    // update J_m and error for gradient
    if(image_id==GRADIENT_ID){
      Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian( GRADIENT_ID);
      J_m += image_jacobian_gradient*Jm_;
      J_r += image_jacobian_gradient*Jr_;
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

  clear();
  //
  // H_cp_.setZero();
  // H_pp_.setZero();
  // b_p_.setZero();
  // dx_c.setZero();
  // dx_p.setZero();

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
    r_idx = measurement->cam_couple_->cam_r_->cam_data_for_ba_->c_idx_*6;
  }

  if(!no_m){
    assert(m_idx < c_size_);
    m_idx = measurement->cam_couple_->cam_m_->cam_data_for_ba_->c_idx_*6;
  }

  assert(p_idx < p_size_);

  // get weight
  float weight = measurement->getWeight();

  // ********* H *********

  // CAM CAM BLOCK

  // m-m
  if(!no_m){
    H_cc_.block<6,6>(m_idx,m_idx).triangularView<Eigen::Upper>() += measurement->J_m_transpose*weight*measurement->J_m;
  }

  // // r-r
  if(!no_r){
    H_cc_.block<6,6>(r_idx,r_idx).triangularView<Eigen::Upper>() += measurement->J_r_transpose*weight*measurement->J_r;
  }

  // m-r
  if(!no_r && !no_m){
    if(m_idx<r_idx){
      H_cc_.block<6,6>(m_idx,r_idx) += measurement->J_m_transpose*weight*measurement->J_r;
    }
    else{
      H_cc_.block<6,6>(r_idx,m_idx) += measurement->J_r_transpose*weight*measurement->J_m;
    }
  }

  // CAM POINT BLOCK

  // m-p
  if(!no_m){
    H_cp_.block<6,1>(m_idx,p_idx) += measurement->J_m_transpose*(weight*measurement->J_d);
  }

  // r-p
  if(!no_r){
    H_cp_.block<6,1>(r_idx,p_idx) += measurement->J_r_transpose*(weight*measurement->J_d);
  }

  // POINT POINT BLOCK

  // p
  H_pp_(p_idx) += measurement->J_d*weight*measurement->J_d;

  // ********* b *********

  // cam segment
  // m
  if(!no_m){
    b_c_.segment<6>(m_idx) += measurement->J_m_transpose*(weight*measurement->error);
  }

  // r
  if(!no_r){
    b_c_.segment<6>(r_idx) += measurement->J_r_transpose*(weight*measurement->error);
  }

  // p
  b_p_(p_idx) += measurement->J_d*weight*measurement->error;

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
  for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->frames_with_active_pts_[i];

    for( ActivePoint* active_pt : cam_r->points_container_->active_points_ ){
      if(active_pt->p_idx_!=-1){

        float new_invdepth = active_pt->invdepth_ + dx_p(active_pt->p_idx_);
        new_invdepth = std::min(new_invdepth,(float)1.0/cam_r->cam_parameters_->min_depth);
        new_invdepth = std::max(new_invdepth,(float)1.0/cam_r->cam_parameters_->max_depth);
        active_pt->updateInvdepthVarAndP( new_invdepth , (1.0/(H_pp_[active_pt->p_idx_]+0.001))+0.001 );
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

  c_size_ = count*6;

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
      J_m += image_jacobian_intensity*Jm_;
      J_d += image_jacobian_intensity*Jd_;


      // update J_m and error for gradient
      if(image_id==GRADIENT_ID){
        Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian( GRADIENT_ID);
        J_m += image_jacobian_gradient*Jm_;
        J_d += image_jacobian_gradient*Jd_;
      }

      J_m_transpose= J_m.transpose();

}
