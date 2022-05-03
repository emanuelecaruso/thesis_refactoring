#include "PointsHandler.h"
#include "dso.h"
#include "CamCouple.h"
#include "epline.h"
#include "CoarseRegions.h"
#include "utils.h"
#include <algorithm>    // std::max

void PointsHandler::sampleCandidates(){

  double t_start=getTime();

  int count = 0;
  int n_pixels_tot = dso_->frame_current_->cam_parameters_->resolution_x*dso_->frame_current_->cam_parameters_->resolution_y;
  int reg_level = trunc(std::log((float)(n_pixels_tot/(pow(4,dso_->parameters_->candidate_level)))/(dso_->parameters_->num_candidates))/std::log(4))+1;
  reg_level=std::min(reg_level,5);
  reg_level=std::max(reg_level,1);

  // get image
  Image<pixelIntensity>* img  = dso_->frame_current_->pyramid_->getMagn(dso_->parameters_->candidate_level);

  while(true){

    int factor = pow(2,reg_level-dso_->parameters_->candidate_level);

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

        cv::minMaxLoc( cropped_image,nullptr,maxVal,nullptr,maxLoc );
        maxLoc->x+=col_coord;
        maxLoc->y+=row_coord;

        if(*maxVal<dso_->parameters_->grad_threshold)
          continue;

        points_taken=true;

        pxl pixel = cvpoint2pxl(*maxLoc);

        img->image_(maxLoc->x,maxLoc->y)=0;

        Candidate* cand = new Candidate(dso_->frame_current_, pixel, dso_->parameters_->candidate_level);

        dso_->frame_current_->points_container_->candidates_.push_back(cand);
        img->setPixel(pixel,0);

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

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Candidates sampled: "+ std::to_string(count) + ", " + std::to_string(deltaTime)+" ms");

}

void PointsHandler::showCandidates(){
  dso_->frame_current_->points_container_->showCandidates();
}
void PointsHandler::showProjectedCandidates(){
  dso_->frame_current_->points_container_->showProjectedCandidates();
}
void PointsHandler::showActivePoints(){
  dso_->frame_current_->points_container_->showActivePoints();
}
void PointsHandler::showCoarseActivePoints(int level){
  dso_->frame_current_->points_container_->showCoarseActivePoints(level);
}
void PointsHandler::showProjectedActivePoints(){
  dso_->frame_current_->points_container_->showProjectedActivePoints();
}
void PointsHandler::showProjectedActivePoints(const std::string& name){
  dso_->frame_current_->points_container_->showProjectedActivePoints(name);
}

void PointsHandler::projectCandidatesOnLastFrame(){

  dso_->frame_current_->points_container_->candidates_projected_.clear();

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    projectCandidates(keyframe, dso_->frame_current_ );
  }
}

void PointsHandler::projectActivePointsOnLastFrame(){

  dso_->frame_current_->points_container_->active_points_projected_.clear();

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    projectActivePoints(keyframe, dso_->frame_current_ );
  }
}

void PointsHandler::generateCoarseActivePoints(){

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    keyframe->points_container_->coarse_regions_->generateCoarseActivePoints();
  }

}

void PointsHandler::projectCandidates(CameraForMapping* cam_r, CameraForMapping* cam_m ){
  CamCouple* cam_couple = new CamCouple(cam_r, cam_m);

  // iterate through candidates
  for(Candidate* cand : (cam_r->points_container_->candidates_)){
    if(cand->invdepth_==-1)
      continue;
    CandidateProjected* cand_proj = new CandidateProjected(cand, cam_couple );
    if( cam_m->image_intensity_->pixelInRange(cand_proj->pixel_) ){
      cam_m->points_container_->candidates_projected_.push_back(cand_proj);
    }
  }

}

void PointsHandler::projectActivePoints(CameraForMapping* cam_r, CameraForMapping* cam_m ){
  CamCouple* cam_couple(new CamCouple(cam_r, cam_m) );

  // iterate through candidates
  for(ActivePoint* active_pt : (cam_r->points_container_->active_points_)){
    if(active_pt->invdepth_==-1)
      continue;
    ActivePointProjected* active_pt_proj(new ActivePointProjected(active_pt, cam_couple ));
    if( cam_m->image_intensity_->pixelInRange(active_pt_proj->pixel_) ){
      cam_m->points_container_->active_points_projected_.push_back(active_pt_proj);
    }
  }
}

void PointsHandler::trackCandidates(bool groundtruth){

  CameraForMapping* last_keyframe = dso_->cameras_container_->getLastActiveKeyframe();

  double t_start=getTime();

  // iterate through keyframes (except last)
  for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];

    if(groundtruth){
      trackCandidatesGroundtruth(keyframe);
    }
    else{
      trackCandidates(keyframe, last_keyframe);
    }
  }

  double t_end=getTime();
  sharedCoutDebug("   - Candidates tracked: " + std::to_string((t_end-t_start)) + " ms");


}

void PointsHandler::trackCandidatesGroundtruth(CameraForMapping* keyframe){

  // iterate through candidates
  for (int i=0; i<keyframe->points_container_->candidates_.size(); i++){
    Candidate* cand = keyframe->points_container_->candidates_.at(i);
    cand->setInvdepthGroundtruth();
  }

}

void PointsHandler::trackCandidates(CameraForMapping* keyframe, CameraForMapping* last_keyframe){

  CamCouple* cam_couple = (new CamCouple(keyframe,last_keyframe));

  // iterate through candidates
  for (int i=keyframe->points_container_->candidates_.size()-1; i>=0; i--){
    Candidate* cand = keyframe->points_container_->candidates_.at(i);

    if( !trackCandidate(cand, cam_couple) ){
      cand->remove();
    }

  }

}

bool PointsHandler::trackCandidate(Candidate* cand, CamCouple* cam_couple){
  // get uv of min and max depth
  Eigen::Vector2f uv_min, uv_max;
  cam_couple->getUv(cand->uv_.x(),cand->uv_.y(),cand->depth_min_,uv_min.x(),uv_min.y());
  cam_couple->getUv(cand->uv_.x(),cand->uv_.y(),cand->depth_max_,uv_max.x(),uv_max.y());

  // check if dp/dinvdpth is too small
  float der = (uv_min-uv_max).norm()/((1.0/cand->depth_min_)-(1.0/cand->depth_max_));
  if(der<dso_->parameters_->der_threshold)
    return false;

  // create epipolar line
  EpipolarLine ep_segment( cam_couple->cam_m_, uv_min, uv_max, cand->level_) ;

  // search pixel in epipolar line
  CandTracker CandTracker(ep_segment, cand, cam_couple, dso_->parameters_);
  bool min_found = CandTracker.searchMin();

  // if min has been found
  if(min_found){
    CandTracker.updateCand();
    return true;
  }
  else{
    return false;
  }
}

float CandTracker::getStandardDeviation( ){


  // GEOMETRIC DISPARITY ERROR
  float g_dot_l; // squared scalar product between direction of gradient and ep_line -> |g| |l| cos(a)
                 // since |g|, |l| =1 -> cos(angle between g and l)
  float angle_g = phase_m;
  float angle_l =ep_segment_.slope2angle();
  if (angle_l<0)
    angle_l+=2*PI;
  float a = radiansSub(angle_g,angle_l);
  float c_a = cos(a);
  g_dot_l=abs(c_a);
  // standard deviation epipolar line (fixed)
  float sd_epline_geo = cand_->cam_->cam_parameters_->pixel_width/4;
  // standard deviation disparity
  float sd_disparity_geo = sd_epline_geo/(g_dot_l+0.1);


  // PHOTOMETRIC ERROR
  // gradient on epline direction
  float magn_g = magn_m;
  float g_p = g_dot_l*magn_g;
  // standard deviation img noise
  // float sd_img_noise = parameters_->sd_img_noise;
  float sd_img_noise = cand_->cam_->cam_parameters_->pixel_width/400;
  // standard deviation photometric
  float sd_disparity_photometric = abs(sd_img_noise/(g_p+0.01));


  // standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric+sd_epline_sampling);
  float standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric);
  // standard_deviation = 2*(sd_disparity_photometric);
  // standard_deviation = 2*(sd_disparity_geo);
  // float standard_deviation = cand_->cam_->cam_parameters_->pixel_width;
  // float standard_deviation = 0.00000000001;

  return standard_deviation;
}


void CandTracker::updateCand(){

  float coord;
  if(ep_segment_.u_or_v)
    coord=uv_.x();
  else
    coord=uv_.y();

  // update depth / invdepth
  float depth;
  cam_couple_->getD1( cand_->uv_.x(), cand_->uv_.y(), depth, coord, ep_segment_.u_or_v );
  cand_->invdepth_=1./depth;
  // std::cout << cand_->invdepth_ << std::endl;

  // get standard deviation
  float standard_deviation = getStandardDeviation();

  // update bounds
  float bound_min, bound_max;
  int sign = pow(-1,(ep_segment_.start>ep_segment_.end));
  float coord_min = coord-sign*standard_deviation;
  float coord_max = coord+sign*standard_deviation;
  cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), cand_->depth_min_, coord_min, ep_segment_.u_or_v);
  cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), cand_->depth_max_, coord_max, ep_segment_.u_or_v);

  cand_->invdepth_var_= (1./cand_->depth_min_)-(1./cand_->depth_max_);


}

bool CandTracker::searchMin( ){
  bool min_segment_reached = false;
  bool min_segment_leaved = false;
  float cost_min = FLT_MAX;

  int repetitive=0;

  // iterate through uvs
  for(int i=0; i<ep_segment_.uvs.size(); i++){
    Eigen::Vector2f uv_m = ep_segment_.uvs[i];
    pxl pixel_m = cam_couple_->cam_m_->uv2pixelCoords( uv_m, cand_->level_);

    if(!cam_couple_->cam_m_->pyramid_->getC(cand_->level_)->pixelInRange(pixel_m))
      continue;

    float cost_magn = getCostMagn(pixel_m);
    float cost_phase=0;
    bool valid = getPhaseCostContribute(pixel_m, uv_m, cost_phase);
    if(!valid)
      return false;

    float cost=cost_magn+cost_phase;
    // float cost=cost_magn;

    if(cost>parameters_->cost_threshold){
      if(min_segment_reached){
        min_segment_leaved=true;
      }

    }
    else{

      if(min_segment_leaved){
        repetitive++;
        return false;
      }
      min_segment_reached=true;


      if(cost<cost_min){
        cost_min=cost;
        uv_=uv_m;
        pixel_=pixel_m;
      }

    }
  }

  if(!min_segment_reached)
    return false;

  // cand_->showCandidate();
  // ep_segment_.showEpipolarWithMin(pixel_, red, cand_->level_, 2);

  return true;
}

float CandTracker::getCostMagn(pxl& pixel){

  pixelIntensity c_m = cam_couple_->cam_m_->pyramid_->getC(cand_->level_)->evalPixelBilinear(pixel);
  magn_m = cam_couple_->cam_m_->pyramid_->getMagn(cand_->level_)->evalPixelBilinear(pixel);

  pixelIntensity c_r = cand_->c_;
  pixelIntensity magn_cd_r = cand_->magn_cd_;

  float cost_c = abs(c_m-c_r);
  float cost_magn_cd = abs(magn_m-magn_cd_r);

  // std::cout << "c_m " << c_m << " c_r " << c_r << " magn_m " << magn_m << " magn_cd_r " << magn_cd_r << "\n";
  // return cost_magn_cd;
  return parameters_->intensity_coeff*cost_c+parameters_->gradient_coeff*cost_magn_cd;
}

bool CandTracker::getPhaseCostContribute(pxl& pixel_m, Eigen::Vector2f& uv_m, float& phase_cost){

  phase_m = cam_couple_->cam_m_->pyramid_->getPhase(cand_->level_)->evalPixelBilinear(pixel_m);

  float pixel_width=cand_->cam_->cam_parameters_->pixel_width/pow(2,cand_->level_);
  float phase_r = cand_->phase_cd_;
  Eigen::Vector2f grad_direction_r {cos(phase_r)*pixel_width,sin(phase_r)*pixel_width};

  // std::cout << "\nphase r: " << phase_r << ", grad dir r:\n" << grad_direction_r << std::endl;
  Eigen::Vector2f tip_to_project = cand_->uv_+grad_direction_r;
  float d1, coord;
  if(ep_segment_.u_or_v)
    coord = uv_m.x();
  else
    coord = uv_m.y();

  cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), d1, coord, ep_segment_.u_or_v);

  Eigen::Vector2f tip_m, direction_m;

  cam_couple_->getUv(tip_to_project.x(), tip_to_project.y(), d1, tip_m.x(), tip_m.y() );

  direction_m=tip_m-uv_m;

  // std::cout << "\n1:\n" << direction_m << " 2:\n " << grad_direction_r << std::endl;

  float phase_m_hat = std::atan2(direction_m.y(),direction_m.x());

  if (phase_m_hat<0)
    phase_m_hat+=2*PI;
  if (phase_m<0)
    phase_m+=2*PI;

  if(abs(phase_m)>10 || abs(phase_m_hat)>10){
    return false;
  }

  // if( (phase_far < phase_m && phase_m < phase_close) || (phase_far > phase_m && phase_m > phase_close) ){
  assert(abs(phase_m_hat)<10);
  assert(abs(phase_m)<10);

  phase_cost= abs(radiansSub(phase_m_hat,phase_m))*parameters_->phase_coeff;
  return true;
}
