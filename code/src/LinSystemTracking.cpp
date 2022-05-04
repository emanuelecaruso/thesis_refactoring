#include "LinSystemTracking.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"

bool MeasTracking::getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel){
  // project active point get pixel of projected active point
  Eigen::Vector2f uv;
  cam_couple->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  cam_couple->cam_m_->uv2pixelCoords( uv, pixel, active_point->level_ );

  if(cam_couple->cam_m_->pyramid_->getC(active_point->level_)->pixelInRange(pixel))
    return true;
  return false;
}

float MeasTracking::getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type){
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

Eigen::Matrix<float,1,2> MeasTracking::getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getCDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getCDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getMagnDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getMagnDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }

  return image_jacobian;
}

bool MeasTracking::init(ActivePoint* active_point, CamCouple* cam_couple){
  assert(active_point->cam_==cam_couple->cam_r_);

  // project active point and get pixel coordinates
  pxl pixel;
  bool pixel_in_cam = getPixelOfProjectedActivePoint(active_point, cam_couple, pixel);
  // if pixel is outside of frustum
  if(!pixel_in_cam){
    valid_ = false;
    return false;
  }

  J_m.setZero();  // initialize J_m
  J_m_transpose.setZero();  // initialize J_m_transpose
  error=0;  // initialize error

  // get Jm_
  // Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_(active_point);
  Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_old_(active_point);

  // update J_m and error for intensity
  Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian(pixel, active_point, cam_couple, INTENSITY_ID);
  J_m += image_jacobian_intensity*Jm_;
  error += getError( pixel,  active_point, cam_couple, INTENSITY_ID);

  // update J_m and error for gradient
  Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian(pixel, active_point, cam_couple, GRADIENT_ID);
  J_m += image_jacobian_gradient*Jm_;
  error += getError( pixel,  active_point, cam_couple, GRADIENT_ID);

  J_m_transpose= J_m.transpose();
  return true;
}



void LinSysTracking::addMeasurement( MeasTracking& measurement ){

  // get weight
  float weight = getWeight(measurement);

  // update H
  H.triangularView<Eigen::Upper>() += measurement.J_m_transpose*weight*measurement.J_m;

  // update b
  b+= measurement.J_m_transpose*weight*measurement.error;

  // update chi
  chi+= measurement.error*weight*measurement.error;
}

void LinSysTracking::updateCameraPose(){

  // get dx
  // dx = H.selfadjointView<Eigen::Upper>().ldlt().solve(-b);
  H=H.selfadjointView<Eigen::Upper>();
  dx = H.completeOrthogonalDecomposition().pseudoInverse()*(-b);

  // update pose
  Eigen::Isometry3f new_guess = (*(dso_->frame_current_->frame_camera_wrt_world_))*v2t_inv(dx);
  dso_->frame_current_->assignPose(new_guess);

}

void LinSysTracking::clear(){
  H.setZero();
  b.setZero();
  dx.setZero();
  chi=0;
}

float LinSysTracking::getWeight(MeasTracking& measurement){
  float weight=1;
  // float weight=1.0/measurement.active_point_->invdepth_var_;

  // huber robustifier
  if(dso_->parameters_->opt_norm==HUBER){
    float huber_threshold=dso_->parameters_->huber_threshold;
    float u = abs(measurement.error);

    if (u<=huber_threshold){
      weight *= 1/huber_threshold;
    }
    else{
      // float rho_der = huberNormDerivative(error,dso_->parameters_->huber_threshold);
      float rho_der = huberNormDerivative(u,dso_->parameters_->huber_threshold);
      float gamma=(1/u)*rho_der;
      weight *= gamma;
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
