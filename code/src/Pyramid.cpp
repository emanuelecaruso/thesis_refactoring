#include "Pyramid.h"
#include "CameraForMapping.h"

void PyramidLevel::init(const Image<pixelIntensity>* img){
  c_= new Image<pixelIntensity>(img);
  c_dx_=c_->compute_sobel_x();
  c_dy_=c_->compute_sobel_y();
  magn_cd_=getMagnitude(c_dx_,c_dy_);
  magn_cd_dx_=magn_cd_->compute_sobel_x();
  magn_cd_dy_=magn_cd_->compute_sobel_y();
  magn_cdd_ =getMagnitude(magn_cd_dx_,magn_cd_dy_);
  phase_cd_=getPhase(c_dx_,c_dy_);
}

void PyramidLevel::init(const Image<pixelIntensity>* img,
                        const Image<pixelIntensity>* c_dx, const Image<pixelIntensity>* c_dy,
                        const Image<pixelIntensity>* cd_magn,
                        const Image<pixelIntensity>* c_dx2, const Image<pixelIntensity>* c_dy2,
                        const Image<pixelIntensity>* cd_magn2,
                        const Image<pixelIntensity>* cd_phase ){
  c_= new Image<pixelIntensity>(img);
  c_dx_ = new Image<pixelIntensity>(c_dx);
  c_dy_ = new Image<pixelIntensity>(c_dy);
  magn_cd_ = new Image<pixelIntensity>(cd_magn);
  magn_cd_dx_ = new Image<pixelIntensity>(c_dx2);
  magn_cd_dy_ = new Image<pixelIntensity>(c_dy2);
  magn_cdd_ = new Image<pixelIntensity>(cd_magn2);
  phase_cd_ = new Image<pixelIntensity>(cd_phase);
}


Image<float>* PyramidLevel::getMagnitude(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* magn (new Image<pixelIntensity>("magn_"+dx->name_+" "+dy->name_));
  cv::magnitude(dx->image_, dy->image_, magn->image_);

  return magn;
}

Image<float>* PyramidLevel::getPhase(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* phase (new Image<pixelIntensity>("phase_"+dx->name_+" "+dy->name_));
  cv::phase(dx->image_, dy->image_, phase->image_);

  return phase;
}


void Pyramid::init(const int levels, const Image<pixelIntensity>* image_intensity, bool pyr_type_){
  assert(levels>0);

  if(pyr_type){

    // clone image
    Image<pixelIntensity>* img (new Image<pixelIntensity>(image_intensity)) ;

    for(int level=0; level<levels_; level++){
      pyramid_levels_.push_back( new PyramidLevel(img) );

      if(level!=levels_){
        cv::resize(img->image_, img->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
      }
    }
  }
  else{
    Image<pixelIntensity>* img (new Image<pixelIntensity>(image_intensity)) ;
    Image<pixelIntensity>* c_dx_=img->compute_sobel_x();
    Image<pixelIntensity>* c_dy_=img->compute_sobel_y();
    Image<pixelIntensity>* magn_cd_= (new Image<pixelIntensity>("magn_"+c_dx_->name_+" "+c_dy_->name_));
    cv::magnitude(c_dx_->image_, c_dy_->image_, magn_cd_->image_);
    Image<pixelIntensity>* magn_cd_dx_=magn_cd_->compute_sobel_x();
    Image<pixelIntensity>* magn_cd_dy_=magn_cd_->compute_sobel_y();
    Image<pixelIntensity>* magn_cdd_= (new Image<pixelIntensity>("magn_"+magn_cd_->name_+" "+magn_cd_->name_));
    cv::magnitude(magn_cd_dx_->image_, magn_cd_dy_->image_, magn_cdd_->image_);
    Image<pixelIntensity>* phase_cd_= (new Image<pixelIntensity>("phase_"+c_dx_->name_+" "+c_dy_->name_));
    cv::phase(c_dx_->image_, c_dy_->image_, phase_cd_->image_);

    for(int level=0; level<levels_; level++){
      pyramid_levels_.push_back( new PyramidLevel(img,c_dx_,c_dy_,magn_cd_,magn_cd_dx_,magn_cd_dy_,magn_cdd_,phase_cd_) );

      if(level!=levels_){
        cv::resize(img->image_, img->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(c_dx_->image_, c_dx_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(c_dy_->image_, c_dy_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(magn_cd_->image_, magn_cd_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(magn_cd_dx_->image_, magn_cd_dx_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(magn_cd_dy_->image_, magn_cd_dy_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(magn_cdd_->image_, magn_cdd_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
        cv::resize(phase_cd_->image_, phase_cd_->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );

      }
    }

  }
}


Image<pixelIntensity>* Pyramid::getC(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  assert(pyramid_levels_[level]);
  return pyramid_levels_[level]->c_;
}

Image<pixelIntensity>* Pyramid::getCDX(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->c_dx_;
}

Image<pixelIntensity>* Pyramid::getCDY(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->c_dy_;
}

Image<pixelIntensity>* Pyramid::getMagn(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_;
}

Image<pixelIntensity>* Pyramid::getMagn2(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cdd_;
}

Image<pixelIntensity>* Pyramid::getMagnDX(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_dx_;
}

Image<pixelIntensity>* Pyramid::getMagnDY(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_dy_;
}

Image<pixelIntensity>* Pyramid::getPhase(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->phase_cd_;
}
