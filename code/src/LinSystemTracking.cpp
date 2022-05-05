#include "LinSystemTracking.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"

bool MeasTracking::getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel, int level){
  // project active point get pixel of projected active point
  Eigen::Vector2f uv;
  cam_couple->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  cam_couple->cam_m_->uv2pixelCoords( uv, pixel, level );

  if(cam_couple->cam_m_->pyramid_->getC(level)->pixelInRange(pixel))
    return true;
  return false;
}

float MeasTracking::getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int level, int image_type){
  float z, z_hat;
  float error;
  // pxl pixel_r = active_point->pixel_/(pow(2,level));
  pxl pixel_r;
  active_point->cam_->uv2pixelCoords( active_point->uv_, pixel_r, level);
  // pixel_r.x() = (int)(active_point->pixel_.x()/pow(2,level));
  // pixel_r.y() = (int)(active_point->pixel_.y()/pow(2,level));
  // std::cout << active_point->pixel_ << " " << pixel_r << std::endl;
  if(image_type==INTENSITY_ID){
    if(level==0){
      z = active_point->c_;
    }
    else{
      z = cam_couple->cam_r_->pyramid_->getC(level)->evalPixelBilinear(pixel_r);
    }
    z_hat = cam_couple->cam_m_->pyramid_->getC(level)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }
  else if(image_type==GRADIENT_ID){
    if(level==0){
      z = active_point->magn_cd_;
    }
    else{
      z = cam_couple->cam_r_->pyramid_->getMagn(level)->evalPixelBilinear(pixel_r);
    }
    z_hat = cam_couple->cam_m_->pyramid_->getMagn(level)->evalPixelBilinear(pixel_m);
    error = (z_hat-z);
  }
  if(abs(error)>1){
    std::cout << "aoooo "<< error << " " << z << " " << active_point->pixel_ << " " << pixel_r << std::endl;
    std::exit(1);
  }
  return error;

}

Eigen::Matrix<float,1,2> MeasTracking::getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int level, Dso* dso, int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << dso->parameters_->intensity_coeff*cam_couple->cam_m_->pyramid_->getCDX(level)->evalPixelBilinear(pixel_m), dso->parameters_->intensity_coeff*cam_couple->cam_m_->pyramid_->getCDY(level)->evalPixelBilinear(pixel_m);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << dso->parameters_->gradient_coeff*cam_couple->cam_m_->pyramid_->getMagnDX(level)->evalPixelBilinear(pixel_m), dso->parameters_->gradient_coeff*cam_couple->cam_m_->pyramid_->getMagnDY(level)->evalPixelBilinear(pixel_m);
  }

  return image_jacobian;
}

bool MeasTracking::init(ActivePoint* active_point, CamCouple* cam_couple, int level, Dso* dso){
  assert(active_point->cam_==cam_couple->cam_r_);

  // project active point and get pixel coordinates
  pxl pixel;
  bool pixel_in_cam = getPixelOfProjectedActivePoint(active_point, cam_couple, pixel, level);
  // if pixel is outside of frustum
  if(!pixel_in_cam){
    valid_ = false;
    return false;
  }

  J_m.setZero();  // initialize J_m
  J_m_transpose.setZero();  // initialize J_m_transpose
  error=0;  // initialize error

  // get Jm_
  Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_(active_point);
  // Eigen::Matrix<float,2,6> Jm_ = cam_couple->getJm_old_(active_point);
  Jm_/=pow(2,level);

  // update J_m and error for intensity
  Eigen::Matrix<float,1,2> image_jacobian_intensity = getImageJacobian(pixel, active_point, cam_couple, level, dso, INTENSITY_ID);
  J_m += image_jacobian_intensity*Jm_;
  error += dso->parameters_->intensity_coeff*getError( pixel,  active_point, cam_couple, level, INTENSITY_ID);


  // // update J_m and error for gradient
  // Eigen::Matrix<float,1,2> image_jacobian_gradient = dso->parameters_->gradient_coeff*getImageJacobian(pixel, active_point, cam_couple, level, dso, GRADIENT_ID);
  // J_m += image_jacobian_gradient*Jm_;
  // error += dso->parameters_->intensity_coeff*getError( pixel,  active_point, cam_couple, level, GRADIENT_ID);

  assert(abs(error)<1);
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
  H = H.selfadjointView<Eigen::Upper>();
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

  float weight_mest;
  // m estimator
  float u = abs(measurement.error);
  if(u>sat_threshold_){
    weight_mest = sat_threshold_;
  }
  else if(dso_->parameters_->opt_norm==HUBER){
    float huber_threshold=dso_->parameters_->huber_threshold;
    if (u<=huber_threshold){
      weight_mest=1/huber_threshold;
    }
    else{
      float rho_der = huberNormDerivative(u,dso_->parameters_->huber_threshold);
      float gamma=(1/u)*rho_der;
      weight_mest = gamma;
    }
  }
  // least square without robustifier
  else if (dso_->parameters_->opt_norm==QUADRATIC){
    weight_mest=1;
  }
  else{
    throw std::invalid_argument( "optimization norm has wrong value" );
  }

  assert(std::isfinite(weight_mest));

  // float weight=1.0/measurement.active_point_->invdepth_var_;

  // // weight
  // float variance = dso_->parameters_->variance;
  // int ni = dso_->parameters_->robustifier_dofs;
  // float weight = (ni+1.0)/(ni+(pow(error,2)/variance));
  //
  // float  weight = weight*gamma;
  float weight = weight_mest;

  // return weight;
  return weight;

}
