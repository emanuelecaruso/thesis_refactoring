#include "wavelet.h"
#include "camera.h"

Image<float>* Wvlt_lvl::getMagnitude(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* magn = new Image<pixelIntensity>("magn_"+dx->name_+" "+dy->name_);

  int width=dx->image_.cols;
  int height=dx->image_.rows;

  magn->initImage(height/2, width/2);

  cv::magnitude(dx->image_, dy->image_, magn->image_);

  return magn;
}

Image<float>* Wvlt_lvl::getPhase(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* phase = new Image<pixelIntensity>("phase_"+dx->name_+" "+dy->name_);

  int width=dx->image_.cols;
  int height=dx->image_.rows;

  phase->initImage(height/2, width/2);

  cv::phase(dx->image_, dy->image_, phase->image_);

  // dx->show(1);
  // dy->show(1);
  // phase->image_/=6.28;
  // phase->show(1);
  //
  // waitkey(0);

  return phase;
}


void Wvlt_lvl::WaveletDecHaar(const Image<pixelIntensity>* img){
  c=new Image<pixelIntensity>("c"+img->name_);
  c_dx=new Image<pixelIntensity>("c_dx"+img->name_);
  c_dy=new Image<pixelIntensity>("c_dy"+img->name_);
  magn_cd=new Image<float>("magn_cd"+img->name_);
  magn_cd_dx=new Image<pixelIntensity>("magn_cd_dx"+img->name_);
  magn_cd_dy=new Image<pixelIntensity>("magn_cd_dy"+img->name_);
  phase_cd=new Image<float>("phase_cd"+img->name_);

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);

  magn_cd->initImage(height/2, width/2);
  c_dx->initImage(height/2, width/2);
  c_dy->initImage(height/2, width/2);
  magn_cd_dx->initImage(height/2, width/2);
  magn_cd_dy->initImage(height/2, width/2);
  phase_cd->initImage(height/2, width/2);

  cv::resize(img->image_, c->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
  // c->image_=img->image_;
  // for (int y=0;y<(height/2);y++)
  // {
  //     for (int x=0; x<(width/2);x++)
  //     {
  //         pixelIntensity c_=(img->image_.at<pixelIntensity>(2*y,2*x)+img->image_.at<pixelIntensity>(2*y,2*x+1)+img->image_.at<pixelIntensity>(2*y+1,2*x)+img->image_.at<pixelIntensity>(2*y+1,2*x+1))*(0.25);
  //         c->image_.at<pixelIntensity>(y,x)=c_;
  //     }
  // }

  c_dx=c->compute_sobel_x();
  c_dy=c->compute_sobel_y();

  magn_cd=getMagnitude(c_dx,c_dy);
  magn_cd_dx=magn_cd->compute_sobel_x();
  magn_cd_dy=magn_cd->compute_sobel_y();

  phase_cd=getPhase(c_dx,c_dy);
}

Wvlt_lvl* Wvlt_dec::getWavLevel(int level){
  return vector_wavelets->at(level);
}

void Wvlt_dec::showWaveletDec(float size){
  showWaveletDec("wavelet decomposition", size);
}

void Wvlt_dec::showWaveletDec(const std::string& name, float size){
  Image<pixelIntensity>* out = new Image<pixelIntensity>(name);
  int cols=image_->image_.cols;
  int rows=image_->image_.rows;


  out->image_= vector_wavelets->at(levels_-1)->c->image_;

  float offset=0.5;

  for (int i=levels_-1; i>=0; i--){
    int cur_rows=rows>>i;
    int cur_cols=cols>>i;
    Wvlt_lvl* wvlt_curr=vector_wavelets->at(i);

    cv::Mat_<pixelIntensity> tmp;
    cv::hconcat(((wvlt_curr->c_dx->image_/8)+offset),(wvlt_curr->c->image_),tmp);

    if(i==levels_-1)
      out->setAllPixels(0); // black image

    Image<pixelIntensity>* magn = new Image<pixelIntensity>(wvlt_curr->c_dy);

    cv::hconcat(out->image_,(magn->image_/8)+offset,out->image_);
    // cv::hconcat(out->image_,(wvlt_curr->c_dx->image_/8)+offset,out->image_);

    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}
