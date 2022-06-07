#include "LinSystem.h"
#include "camera.h"
#include "PointsContainer.h"
#include "dso.h"
#include "CamCouple.h"

bool Meas::getPixelOfProjectedActivePoint(ActivePoint* active_point){
  // project active point get pixel of projected active point
  Eigen::Vector2f uv;
  bool valid = cam_couple_->getUv( active_point->uv_.x(),active_point->uv_.y(),1./active_point->invdepth_,uv.x(),uv.y() );
  if( !valid )
    return false;

  if(cam_couple_->cam_m_->pyramid_==nullptr)
    std::cout << "PORCODIO " << cam_couple_->cam_m_->name_ << std::endl;
  assert(cam_couple_->cam_m_->pyramid_!=nullptr);
  cam_couple_->cam_m_->uv2pixelCoords( uv, pixel_, level_ );
  bool pixel_in_range = cam_couple_->cam_m_->pyramid_->getC(level_)->pixelInRange(pixel_);

  if(!pixel_in_range){
    valid_=false;
    return false;
  }

  return true;
}

float Meas::getErrorIntensity(float z, float z_hat ){
  float error = intensity_coeff_ba*cam_couple_->getErrorIntensity(z,z_hat);
  return error;
}
float Meas::getErrorGradient(float z, float z_hat ){
  float error = gradient_coeff_ba*cam_couple_->getErrorGradient(z, z_hat);
  return error;
}


float Meas::getError( ActivePoint* active_point, int image_type){

  pxl pixel_r;
  active_point->cam_->uv2pixelCoords( active_point->uv_, pixel_r, level_);

  if(image_type==INTENSITY_ID){

    float z = active_point->c_level_vec_[level_];
    float z_hat = cam_couple_->cam_m_->pyramid_->getC(level_)->evalPixelBilinear(pixel_);

    float error = getErrorIntensity( z, z_hat );
    return error;


  }
  else if(image_type==GRADIENT_ID){
    float z = active_point->magn_cd_level_vec_[level_];
    float z_hat = cam_couple_->cam_m_->pyramid_->getMagn(level_)->evalPixelBilinear(pixel_);

    float error = getErrorGradient(z, z_hat );
    return error;

  }

}

// float Meas::getError( ActivePoint* active_point, int image_type){
//   float z, z_hat;
//   // pxl pixel_r = active_point->pixel_/(pow(2,level_));
//   pxl pixel_r;
//   active_point->cam_->uv2pixelCoords( active_point->uv_, pixel_r, level_);
//
//   if(image_type==INTENSITY_ID){
//     if(level_==0){
//       z = active_point->c_;
//     }
//     else{
//       z = cam_couple_->cam_r_->pyramid_->getC(level_)->evalPixelBilinear(pixel_r);
//     }
//     z_hat = cam_couple_->cam_m_->pyramid_->getC(level_)->evalPixelBilinear(pixel_);
//
//     float error = getErrorIntensity( z, z_hat, active_point);
//     return error;
//
//
//   }
//   else if(image_type==GRADIENT_ID){
//     if(level_==0){
//       z = active_point->magn_cd_;
//     }
//     else{
//       z = cam_couple_->cam_r_->pyramid_->getMagn(level_)->evalPixelBilinear(pixel_r);
//     }
//     z_hat = cam_couple_->cam_m_->pyramid_->getMagn(level_)->evalPixelBilinear(pixel_);
//
//     float error = getErrorGradient(z, z_hat, active_point);
//     return error;
//
//   }
//
// }

Eigen::Matrix<float,1,2> Meas::getImageJacobian( int image_type){

  Eigen::Matrix<float,1,2> image_jacobian;

  if(image_type==INTENSITY_ID){
    image_jacobian << intensity_coeff_ba*cam_couple_->cam_m_->pyramid_->getCDX(level_)->evalPixelBilinear(pixel_),
                      intensity_coeff_ba*cam_couple_->cam_m_->pyramid_->getCDY(level_)->evalPixelBilinear(pixel_);
  }
  else if(image_type==GRADIENT_ID){
    image_jacobian << gradient_coeff_ba*cam_couple_->cam_m_->pyramid_->getMagnDX(level_)->evalPixelBilinear(pixel_),
                      gradient_coeff_ba*cam_couple_->cam_m_->pyramid_->getMagnDY(level_)->evalPixelBilinear(pixel_);
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
  float error_intensity = getError( active_point, INTENSITY_ID);
  if( abs(error_intensity) > active_point->cost_threshold_ba_intensity_*(1+(level_*10)) ){
    // std::cout << abs(error_intensity) << " " << active_point->cost_threshold_ba_intensity_ << std::endl;
    occlusion_ = true;
    return false;
  }
  error += error_intensity;

  if(image_id==GRADIENT_ID){
    float error_gradient = getError( active_point, GRADIENT_ID);
    if( abs(error_gradient) > active_point->cost_threshold_ba_gradient_*(1+(level_*10)) ){
      // std::cout << abs(error_gradient) << " " << active_point->cost_threshold_ba_gradient_ << std::endl;
      occlusion_ = true;
      return false;
    }
    error += error_gradient;
  }

  // if( abs(error) > (active_point->cost_threshold_ba_intensity_+active_point->cost_threshold_ba_gradient_)*(1+(level_*6)) ){
  //   // std::cout << abs(error_intensity) << " " << active_point->cost_threshold_ba_intensity_ << std::endl;
  //   occlusion_ = true;
  //   return false;
  // }

  return true;
}



float Meas::getWeight(){

  float weight_mest;
  // m estimator
  float u = abs(error);
  if(u>sat_threshold){
    weight_mest = sat_threshold/u;
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

  assert(std::isfinite(weight_mest));

  // float  weight = weight_mest*(1.0/var_);
  float weight = weight_mest;

  // return weight;
  return weight;

}

void LinSysBlocks::resize(int c_size, int p_size){
  c_size_=c_size;
  p_size_=p_size;

  H_cc_.resize(c_size,c_size);
  H_cp_.resize(c_size,p_size);
  H_pp_.resize(p_size);
  b_c_.resize(c_size);
  b_p_.resize(p_size);
  omega_.resize(p_size);

  H_cc_.setZero();
  H_cp_.setZero();
  H_pp_.setZero();
  b_c_.setZero();
  b_p_.setZero();
  omega_.setZero();

}


void LinSysBlocks::reset(){
  c_size_=0;
  p_size_=0;
  H_cc_.resize(0,0);
  H_cp_.resize(0,0);
  H_pp_.resize(0);
  b_c_.resize(0);
  b_p_.resize(0);
}

void LinSysBlocks::clear(){
  H_cc_.setZero();
  b_c_.setZero();
  H_cp_.setZero();
  H_pp_.setZero();
  b_p_.setZero();
  omega_.setZero();

}


bool LinSysBlocks::visualizeH(){
  Image<colorRGB>* img_H = new Image<colorRGB>("H");
  int size = c_size_+p_size_;
  if (size == 0)
    return false;
  // img_H->initImage(c_size_,c_size_);
  img_H->initImage(size,size);
  img_H->setAllPixels( white);

  // pose pose block
  for(int i=0; i<c_size_; i++){
    for(int j=0; j<c_size_; j++){
      float val;
      if(i<j){
        val=H_cc_(i,j);
      }
      else{
        val=H_cc_(j,i);
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
  for(int i=0; i<c_size_; i++){
    for(int j=0; j<p_size_; j++){
      if ((H_cp_)(i,j)!=0){
        img_H->setPixel(i,c_size_+j, green);
      }
      else{
        img_H->setPixel(i,c_size_+j, white);

      }
    }
  }

  // point pose block
  for(int i=0; i<p_size_; i++){
    for(int j=0; j<c_size_; j++){
      if ((H_cp_)(j,i)!=0){
        img_H->setPixel(i+c_size_,j, green);
      }
      else{
        img_H->setPixel(i+c_size_,j, white);

      }
    }
  }

  // point point block
  for(int i=0; i<p_size_; i++){
    if (H_pp_(i)!=0){
      img_H->setPixel(i+c_size_,i+c_size_, blue);
    }
    else{
      img_H->setPixel(i+c_size_,i+c_size_, white);

    }

  }

  img_H->show(1);
  waitkey(0);
  delete img_H;

}



bool LinSysBlocks::visualizeB(){
  Image<colorRGB>* img_B = new Image<colorRGB>("B");
  int size = c_size_+p_size_;
  if (size == 0)
    return false;
  // img_H->initImage(c_size_,c_size_);
  img_B->initImage(size,1);
  img_B->setAllPixels( white);

  // pose segment
  for(int i=0; i<c_size_; i++){
    float val;
    val=b_c_(i);

    if (val!=0){
      img_B->setPixel(i,0, red);
    }
    else{
      img_B->setPixel(i,0, white);
    }

  }
  // point point block
  for(int i=0; i<p_size_; i++){
    if (b_p_(i)!=0){
      img_B->setPixel(i+c_size_,0, blue);
    }
    else{
      img_B->setPixel(i+c_size_,0, white);

    }

  }
}
