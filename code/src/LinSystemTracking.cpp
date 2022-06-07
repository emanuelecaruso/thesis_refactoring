#include "LinSystemTracking.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"


void MeasTracking::loadJacobians(ActivePoint* active_point){

    J_m.setZero();  // initialize J_m

    // get Jm_
    Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_(active_point,level_);
    // Eigen::Matrix<float,2,6> Jm_ = cam_couple_->getJm_old_(active_point);

    // update J_m and error for intensity
    Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian( INTENSITY_ID);
    J_m.head<6>() += image_jacobian_intensity*Jm_;


    // update J_m and error for gradient
    if(image_id==GRADIENT_ID){
      Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian( GRADIENT_ID);
      J_m.head<6>() += image_jacobian_gradient*Jm_;
    }

    if(J_SZ==8){
      J_m.tail<2>() += cam_couple_->getJm_exposure_(active_point, level_);
    }

    J_m_transpose= J_m.transpose();
}

void LinSysTracking::addMeasurement( MeasTracking& measurement ){

  // get weight
  float weight = measurement.getWeight();

  // update H
  H.triangularView<Eigen::Upper>() += measurement.J_m_transpose*weight*measurement.J_m;

  // update b
  b+= measurement.J_m_transpose*weight*measurement.error;

  // update chi
  chi+= measurement.error*weight*measurement.error;
}

void LinSysTracking::updateCameraPose(){

  // exposure priors
  if(J_SZ==8){
    H(6,6) += 2*lambda_a*abs(dso_->frame_current_->a_exposure_)+damp_exposure;
    H(7,7) += 2*lambda_b*abs(dso_->frame_current_->b_exposure_)+damp_exposure;
    // H(6,6) = FLT_MAX;
    // H(7,7) = FLT_MAX;
  }


  // get dx
  // dx = H.selfadjointView<Eigen::Upper>().ldlt().solve(-b);
  H = H.selfadjointView<Eigen::Upper>();
  dx = H.completeOrthogonalDecomposition().pseudoInverse()*(-b);
  Vector6f dx_pose = dx.head<6>();

  // update pose
  Eigen::Isometry3f new_guess = (*(dso_->frame_current_->frame_camera_wrt_world_))*v2t_inv(dx_pose);
  dso_->frame_current_->assignPose(new_guess);

  if(J_SZ==8){
    // update exposure parameters
    dso_->frame_current_->a_exposure_+= dx(6);
    dso_->frame_current_->b_exposure_+= dx(7);
  }

}


void LinSysTracking::clear(){
  H.setZero();
  b.setZero();
  dx.setZero();
  chi=0;
}
