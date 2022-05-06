#include "LinSystem.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"

bool Meas::getPixelOfProjectedActivePoint(ActivePoint* active_point){
  // project active point get pixel of projected active point
  Eigen::Vector2f uv;
  cam_couple_->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  cam_couple_->cam_m_->uv2pixelCoords( uv, pixel_, level_ );
  bool pixel_in_range = cam_couple_->cam_m_->pyramid_->getC(level_)->pixelInRange(pixel_);

  if(!pixel_in_range){
    valid_=false;
    return false;
  }

  return true;
}

float Meas::getError( ActivePoint* active_point, int image_type){
  float z, z_hat;
  float error;
  // pxl pixel_r = active_point->pixel_/(pow(2,level_));
  pxl pixel_r;
  active_point->cam_->uv2pixelCoords( active_point->uv_, pixel_r, level_);
  // pixel_r.x() = (int)(active_point->pixel_.x()/pow(2,level_));
  // pixel_r.y() = (int)(active_point->pixel_.y()/pow(2,level_));
  // std::cout << active_point->pixel_ << " " << pixel_r << std::endl;
  if(image_type==INTENSITY_ID){
    if(level_==0){
      z = active_point->c_;
    }
    else{
      z = cam_couple_->cam_r_->pyramid_->getC(level_)->evalPixelBilinear(pixel_r);
    }
    z_hat = cam_couple_->cam_m_->pyramid_->getC(level_)->evalPixelBilinear(pixel_);
    error = intensity_coeff*(z_hat-z);
  }
  else if(image_type==GRADIENT_ID){
    if(level_==0){
      z = active_point->magn_cd_;
    }
    else{
      z = cam_couple_->cam_r_->pyramid_->getMagn(level_)->evalPixelBilinear(pixel_r);
    }
    z_hat = cam_couple_->cam_m_->pyramid_->getMagn(level_)->evalPixelBilinear(pixel_);
    error = gradient_coeff*(z_hat-z);
  }
  if(abs(error)>1){
    std::cout << "error "<< error << " level " << level_ << " z " << z << " pixel r " << active_point->pixel_ << " pixel m " << pixel_ <<  " z hat " << z_hat << " image_type " << image_type << " rows " << cam_couple_->cam_m_->pyramid_->getC(level_)->image_.rows << " cols " << cam_couple_->cam_m_->pyramid_->getC(level_)->image_.cols << " valid " << valid_ << std::endl;
    std::exit(1);
  }
  return error;

}

Eigen::Matrix<float,1,2> Meas::getImageJacobian( int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << intensity_coeff*cam_couple_->cam_m_->pyramid_->getCDX(level_)->evalPixelBilinear(pixel_),
                      intensity_coeff*cam_couple_->cam_m_->pyramid_->getCDY(level_)->evalPixelBilinear(pixel_);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << gradient_coeff*cam_couple_->cam_m_->pyramid_->getMagnDX(level_)->evalPixelBilinear(pixel_),
                      gradient_coeff*cam_couple_->cam_m_->pyramid_->getMagnDY(level_)->evalPixelBilinear(pixel_);
  }

  return image_jacobian;
}

bool Meas::init(ActivePoint* active_point){
  assert(active_point->cam_==cam_couple_->cam_r_);

  // project active point and get pixel coordinates
  bool pixel_in_cam = getPixelOfProjectedActivePoint(active_point);
  // if pixel is outside of frustum
  if(!pixel_in_cam){
    valid_ = false;
    return false;
  }

  error=0;  // initialize error
  error += getError( active_point, INTENSITY_ID);

  if(image_id==GRADIENT_ID){
    error += getError( active_point, GRADIENT_ID);
  }

  // control on error
  if(abs(error)>chi_occlusion_threshold){
    occlusion_ = true;
    return false;
  }

  return true;
}



float Meas::getWeight(){

  float weight_mest;
  // m estimator
  float u = abs(error);
  if(u>sat_threshold){
    weight_mest = sat_threshold;
  }
  else{
    switch (opt_norm)
    {
      case HUBER:{
        if (u<=huber_threshold){
          weight_mest=1./huber_threshold;
        }
        else{
          float rho_der = huberNormDerivative(u,huber_threshold);
          float gamma=(1./u)*rho_der;
          weight_mest = gamma;
        }
        break;
      }
      case QUADRATIC:{
        weight_mest=1;
        break;
      }
      default:{
        throw std::invalid_argument( "optimization norm has wrong value" );
      }
    }

  }

  // std::cout << huber_threshold << std::endl;
  assert(std::isfinite(weight_mest));

  // float weight=1.0/measurement.active_point_->invdepth_var_;

  // // weight
  // float variance = variance;
  // int ni = robustifier_dofs;
  // float weight = (ni+1.0)/(ni+(pow(error,2)/variance));
  //
  // float  weight = weight*gamma;
  float weight = weight_mest;

  // return weight;
  return weight;

}
