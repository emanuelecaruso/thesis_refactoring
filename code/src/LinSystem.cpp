#include "LinSystem.h"
#include "camera.h"
#include "PointsContainer.h"
#include "CamCouple.h"

bool MeasTracking::getPixelOfProjectedActivePoint(std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, pxl& pixel){
  // project active point and get pixel of projected active point
  Eigen::Vector2f uv;
  cam_couple->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  cam_couple->cam_m_->uv2pixelCoords( uv, pixel, active_point->level_ );

  if(cam_couple->cam_m_->pyramid_->getC(active_point->level_)->pixelInRange(pixel))
    return true;
  return false;
}

float MeasTracking::getError(pxl& pixel_m, std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, int image_type){
  float z, z_hat;
  float error;
  if(image_type==INTENSITY_ID){
    z = active_point->intensity_;
    z_hat = cam_couple->cam_m_->pyramid_->getC(active_point->level_)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }
  else if(image_type==GRADIENT_ID){
    z = active_point->grad_magnitude_;
    z_hat = cam_couple->cam_m_->pyramid_->getMagn(active_point->level_)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }

}

Eigen::Matrix<float,1,2> MeasTracking::getImageJacobian(pxl& pixel_m, std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getCDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getCDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << cam_couple->cam_m_->pyramid_->getMagnDX(active_point->level_)->evalPixelBilinear(pixel_m), cam_couple->cam_m_->pyramid_->getMagnDY(active_point->level_)->evalPixelBilinear(pixel_m);
  }

  return image_jacobian;
}

bool MeasTracking::init(std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple){
  assert(active_point->cam_==cam_couple->cam_r_);

  // project active point and get pixel coordinates
  pxl pixel;
  bool pixel_in_cam = getPixelOfProjectedActivePoint(active_point, cam_couple, pixel);
  // if pixel is outside of frustum
  if(!pixel_in_cam){
    valid = false;
    return false;
  }

  J_m.setZero();  // initialize J_m
  error=0;  // initialize error

  // get Jm_
  Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_(active_point);

  // update J_m and error for intensity
  Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian(pixel, active_point, cam_couple, INTENSITY_ID);
  error += getError( pixel,  active_point, cam_couple, INTENSITY_ID);

  // // update J_m and error for gradient
  // Eigen::Matrix<float,1,2> image_jacobian_gradient = getImageJacobian(pixel, active_point, cam_couple, GRADIENT_ID);
  // error += getError( pixel,  active_point, cam_couple, GRADIENT_ID);

}

void LinSysTracking::addMeasurement( std::shared_ptr<MeasTracking> measurement ){

}

void LinSysTracking::updateCameraPose(){
  
}
