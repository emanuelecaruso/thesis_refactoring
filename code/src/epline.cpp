// #include "mapper.h"
#include "epline.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <cstdlib>
#include <chrono>
#include <CameraForMapping.h>

// void EpipolarLine::printMembers() const {
//
//   sharedCout("\n"+cam->name_+", epipolar line:");
//   sharedCout("slope: "+std::to_string(slope));
//   sharedCout("c0: "+std::to_string(c0));
//   sharedCout("u_or_v: "+std::to_string(u_or_v));
//   sharedCout("start: "+std::to_string(start));
//   sharedCout("end: "+std::to_string(end));
// }
//
void EpipolarLine::coordToUV(float& coord, Eigen::Vector2f& uv){
  if (u_or_v){
    uv.x()=coord;
    uv.y()= slope*coord+c0;
  }
  else{
    uv.y()= coord;
    uv.x()= coord/slope+c0;
  }
}

void EpipolarLine::UVToCoord(Eigen::Vector2f& uv, float& coord){
  if(u_or_v)
    coord=uv.x();
  else
    coord=uv.y();
}



void EpipolarLine::lineTraverse(int level)
{

    float distance = abs(end-start);

    float pixel_width=cam->cam_parameters_->width/(float)cam->cam_parameters_->resolution_x;
    // float pixel_width=cam->cam_parameters_->width/((float)cam->cam_parameters_->resolution_x/pow(2,level));

    int n_uvs= (distance/pixel_width)+2;
    // int n_uvs= ((distance/pixel_width)+1)/(level+2);
    // std::cout << n_uvs << std::endl;

    uvs.resize(n_uvs);

    for (int i=0; i<n_uvs; i++){
      float ratio = (float)i/(n_uvs-1);
      float coord = (1-ratio)*start+(ratio)*end;

      Eigen::Vector2f uv;
      coordToUV(coord,uv);
      uvs.at(i)= uv;
    }
}

float EpipolarLine::slope2angle(){
  float angle = std::atan2(slope,1);
  if (angle>PI || angle<-PI){
    std::cout << "AOOOCHECAZZ " << angle << " " << slope << std::endl;
    exit(1);
  }
  return angle;
}

// show
void EpipolarLine::showEpipolar(int level,float size){
    Image<colorRGB>* image_intensity_new = createEpipolarImg(level);
    image_intensity_new->show(size*(pow(2,level+1)));
    cv::waitKey(0);
    delete image_intensity_new;
}

void EpipolarLine::showEpipolarWithMin(pxl& pixel, const colorRGB& color, int level,float size){
    Image<colorRGB>* image_intensity_new = createEpipolarImg(level);
    image_intensity_new->setPixel(pixel,color);
    // image_intensity_new->show(size*(pow(2,level)), cam->name_);
    image_intensity_new->show(size*(pow(2,level)), cam->name_);
    cv::waitKey(0);
    delete image_intensity_new;
}


void EpipolarLine::drawEpipolar(Image<colorRGB>* img, const colorRGB& color, int level){

    if (!uvs.empty())
      for( int i=0; i<uvs.size(); i++){
        pxl pixel;

        cam->uv2pixelCoords(uvs[i],pixel,level);

        // if(i==0)
        //   image_intensity_new->setPixel(pixel, blue);
        // else if (i==uvs.size()-1)
        //   image_intensity_new->setPixel(pixel, red);
        // else
        img->setPixel(pixel, color);
      }

}


// create imgs to show
Image<colorRGB>* EpipolarLine::createEpipolarImg(const std::string& name, int level){

    Image<colorRGB>* image_intensity_new = new Image<colorRGB> (cam->name_) ;

    image_intensity_new =cam->pyramid_->getC(level)->returnColoredImgFromIntensityImg("epipolar") ;

    drawEpipolar(image_intensity_new, green, level);

    return image_intensity_new;
}

Image<colorRGB>* EpipolarLine::createEpipolarImg(int level){
  return createEpipolarImg("epipolar_"+cam->name_,level);
}
