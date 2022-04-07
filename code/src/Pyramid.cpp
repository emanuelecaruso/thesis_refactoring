#include "Pyramid.h"
#include "CameraForMapping.h"

void PyramidLevel::init(std::shared_ptr<Image<pixelIntensity>> img){
  c_=img;
  c_dx_=c_->compute_sobel_x();
  c_dy_=c_->compute_sobel_y();
  magn_cd_=getMagnitude(c_dx_,c_dy_);
  magn_cd_dx_=magn_cd_->compute_sobel_x();
  magn_cd_dy_=magn_cd_->compute_sobel_y();
  phase_cd_=getPhase(c_dx_,c_dy_);
}


std::shared_ptr<Image<float>> PyramidLevel::getMagnitude(std::shared_ptr<Image<pixelIntensity>> dx, std::shared_ptr<Image<pixelIntensity>> dy )
{
  std::shared_ptr<Image<pixelIntensity>> magn (new Image<pixelIntensity>("magn_"+dx->name_+" "+dy->name_));
  cv::magnitude(dx->image_, dy->image_, magn->image_);

  return magn;
}

std::shared_ptr<Image<float>> PyramidLevel::getPhase(std::shared_ptr<Image<pixelIntensity>> dx, std::shared_ptr<Image<pixelIntensity>> dy )
{
  std::shared_ptr<Image<pixelIntensity>> phase (new Image<pixelIntensity>("phase_"+dx->name_+" "+dy->name_));
  cv::phase(dx->image_, dy->image_, phase->image_);

  return phase;
}


void Pyramid::init(const int levels, std::shared_ptr<Image<pixelIntensity>> image_intensity){
  assert(levels>0);

  // clone image
  std::shared_ptr<Image<pixelIntensity>> img (new Image<pixelIntensity>(image_intensity)) ;

  for(int level=0; level<levels_; level++){
    // pyramid_levels_[level]=std::shared_ptr<PyramidLevel>(new PyramidLevel(img));
    //
    // if(level!=levels_)
    //   cv::resize(img->image_, img->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
  }
}
