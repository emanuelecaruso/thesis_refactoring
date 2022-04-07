#include "PointsHandler.h"
#include "dso.h"
#include <algorithm>    // std::max

void PointsHandler::sampleCandidates(){
  int count = 0;
  int reg_level = trunc(std::log((float)(dso_->frame_current_->cam_parameters_->resolution_x*dso_->frame_current_->cam_parameters_->resolution_y)/(dso_->parameters_->num_candidates))/std::log(4))+1;
  reg_level=std::min(reg_level,5);
  reg_level=std::max(reg_level,1);

  // get image
  std::shared_ptr<Image<pixelIntensity>> img(new  Image<pixelIntensity>(dso_->frame_current_->pyramid_->getMagn(dso_->parameters_->candidate_level)) );

  while(true){

    int factor = pow(2,reg_level);

    assert(!(img->image_.rows%factor));
    assert(!(img->image_.cols%factor));

    int n_regions_x = img->image_.cols/factor;
    int n_regions_y = img->image_.rows/factor;
    int region_width = factor;
    int region_height = factor;

    bool points_taken=false;

    int num_cand_taken = 0;
    for (int row=0; row<n_regions_y; row++){
      for (int col=0; col<n_regions_x; col++){
        int row_coord = row*region_height;
        int col_coord = col*region_width;

        cv::Mat cropped_image (img->image_, cv::Rect(col_coord,row_coord,factor,factor));

        cv::Point2i* maxLoc = new cv::Point2i;
        double* maxVal = new double;

        cv::minMaxLoc 	( cropped_image,nullptr,maxVal,nullptr,maxLoc );
        maxLoc->x+=col_coord;
        maxLoc->y+=row_coord;

        if(*maxVal<dso_->parameters_->grad_threshold)
          continue;

        points_taken=true;

        pxl pixel = cvpoint2pxl(*maxLoc);

        img->image_(maxLoc->x,maxLoc->y)=0;
        img->setPixel(pixel,0);

        std::shared_ptr<Candidate> cand(new Candidate(dso_->frame_current_, pixel, dso_->parameters_->candidate_level));

        dso_->frame_current_->points_container_->candidates_.push_back(cand);

        delete maxLoc;
        delete maxVal;

        num_cand_taken++;
      }
    }

    count+=num_cand_taken;
    int num_cand_less = dso_->parameters_->num_candidates-count;

    if(!points_taken || num_cand_less<0)
      break;

    float ratio = ((float)num_cand_less)/(float)num_cand_taken;
    int diff = std::log(ratio)/std::log(4);
    reg_level+=diff;
    reg_level=std::min(reg_level,5);
    reg_level=std::max(reg_level,1);
  }


}

void PointsHandler::showCandidates(){

    double alpha = 1;

    std::string name = dso_->frame_current_->name_+" , "+std::to_string(dso_->frame_current_->points_container_->candidates_.size())+" candidates";
    Image<colorRGB>* show_img = dso_->frame_current_->pyramid_->getC(dso_->parameters_->candidate_level)->returnColoredImgFromIntensityImg(name);

    for(int i=0; i<dso_->frame_current_->points_container_->candidates_.size(); i++){
      std::shared_ptr<Candidate> candidate = dso_->frame_current_->points_container_->candidates_[i];
      // get level
      int level = candidate->level_;

      pxl pixel= candidate->pixel_;
      pixel*=pow(2,level);

      // compute corners
      cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level),pow(2,level));

      colorRGB color = black;
      // if(candidate->one_min_)
      //   color = invdepthToRgb(candidate->invdepth_);

      show_img->drawCircle( color, pixel, 1, 2);

    }

    show_img->show(1);
    cv::waitKey(0);
    delete show_img;
    // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}
