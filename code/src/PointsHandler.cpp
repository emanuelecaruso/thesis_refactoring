#include "PointsHandler.h"
#include "dso.h"
#include "CamCouple.h"
#include "epline.h"
#include "utils.h"
#include <algorithm>    // std::max

bool PointsHandler::sampleCandidates(){

  double t_start=getTime();

  int count = 0;
  int n_pixels_tot = dso_->frame_current_->cam_parameters_->resolution_x*dso_->frame_current_->cam_parameters_->resolution_y/(pow(4,candidate_level));
  // int reg_level = trunc(std::log( (float)(n_pixels_tot)/(num_candidates))/std::log(4))+1;
  // int reg_level = trunc(std::log( (float)(n_pixels_tot)/(num_candidates))/std::log(4));
  int num = num_candidates;
  // int num =(max_num_active_points-dso_->frame_current_->points_container_->active_points_projected_.size())+1;

  int reg_level = trunc(std::log( (float)(n_pixels_tot)/num)/std::log(4))+1;


  // std::cout << "reg_level " << reg_level << std::endl;
  reg_level=std::min(reg_level,5);
  reg_level=std::max(reg_level,1);

  // get image
  Image<pixelIntensity>* img_magn_coarse  = new Image<pixelIntensity>(dso_->frame_current_->pyramid_->getMagn(candidate_level));
  Image<pixelIntensity>* img_magn2_coarse  = new Image<pixelIntensity>(dso_->frame_current_->pyramid_->getMagn2(candidate_level));
  Image<pixelIntensity>* img_magn  = new Image<pixelIntensity>(dso_->frame_current_->pyramid_->getMagn(candidate_level));

  float factor_coarse =1.0/(float)pow(2,coarsest_lev_magn-1);
  int factor = pow(2,reg_level-candidate_level);

  // cv::resize(img_magn_coarse->image_, img_magn_coarse->image_, cv::Size(), factor_coarse, factor_coarse, cv::INTER_LINEAR );
  // cv::resize(img_magn2_coarse->image_, img_magn2_coarse->image_, cv::Size(), factor_coarse, factor_coarse, cv::INTER_LINEAR );

  // img_magn2_coarse->show(3);
  // cv::waitKey(0);

  while(true){


    assert(!(img_magn->image_.rows%factor));
    assert(!(img_magn->image_.cols%factor));

    int n_regions_x = img_magn->image_.cols/factor;
    int n_regions_y = img_magn->image_.rows/factor;
    int region_width = factor;
    int region_height = factor;

    bool points_taken=false;

    int num_cand_taken = 0;
    for (int row=1; row<n_regions_y-1; row++){
      for (int col=1; col<n_regions_x-1; col++){

        int row_coord = row*region_height;
        int col_coord = col*region_width;

        cv::Mat cropped_image_magn (img_magn->image_, cv::Rect(col_coord,row_coord,factor,factor));


        cv::Point2i* maxLoc = new cv::Point2i;
        double* maxVal = new double;

        cv::minMaxLoc( cropped_image_magn,nullptr,maxVal,nullptr,maxLoc );

        maxLoc->x+=col_coord;
        maxLoc->y+=row_coord;

        if(*maxVal<grad_threshold)
          continue;

        points_taken=true;

        pxl pixel = cvpoint2pxl(*maxLoc);

        Candidate* cand = new Candidate(dso_->frame_current_, pixel, candidate_level);

        // pxl pixel_coarse =pixel*factor_coarse;
        // float mean_magn = img_magn_coarse->evalPixelBilinear(pixel_coarse);
        // float mean_magn2 = img_magn2_coarse->evalPixelBilinear(pixel_coarse);



        cv::Mat cropped_image_magn1 (img_magn_coarse->image_, cv::Rect(col_coord,row_coord,factor,factor));
        cv::Mat cropped_image_magn2 (img_magn2_coarse->image_, cv::Rect(col_coord,row_coord,factor,factor));
        float mean_magn = cv::mean(cropped_image_magn1)[0];
        float mean_magn2 = cv::mean(cropped_image_magn2)[0];


        if(adaptive_thresh){
          if(magn_mean){
            cand->cost_threshold_mapping_=
            intensity_coeff_mapping*(mean_magn*mean_magn_coeff_mapping + g_th_intensity_mapping ) +
            gradient_coeff_mapping*(mean_magn2*mean_magn_coeff_mapping + g_th_gradient_mapping );

            cand->cost_threshold_ba_=
            intensity_coeff_ba*(mean_magn*mean_magn_coeff_ba + g_th_intensity_ba ) +
            gradient_coeff_ba*(mean_magn2*mean_magn_coeff_ba + g_th_gradient_ba );          }
          else{
            cand->cost_threshold_mapping_=
            intensity_coeff_mapping*((cand->magn_cd_)*mean_magn_coeff_mapping + g_th_intensity_mapping ) +
            gradient_coeff_mapping*((cand->magn_cd2_)*mean_magn_coeff_mapping + g_th_gradient_mapping );

            cand->cost_threshold_ba_=
            intensity_coeff_ba*((cand->magn_cd_)*mean_magn_coeff_ba + g_th_intensity_ba ) +
            gradient_coeff_ba*((cand->magn_cd2_)*mean_magn_coeff_ba + g_th_gradient_ba );            // cand->cost_threshold_mapping_=( intensity_coeff*(cand->c_) +
            // gradient_coeff*(cand->magn_cd_) )*mean_magn_coeff;
          }
        }
        else{
          cand->cost_threshold_mapping_=(intensity_coeff_mapping+gradient_coeff_mapping)*fixed_thresh;
          cand->cost_threshold_ba_=(intensity_coeff_ba+gradient_coeff_ba)*fixed_thresh;
        }

        dso_->frame_current_->points_container_->candidates_.push_back(cand);
        img_magn->setPixel(pixel,0);

        delete maxLoc;
        delete maxVal;

        num_cand_taken++;
      }
    }

    count+=num_cand_taken;
    int num_cand_less = num-count;

    if(!points_taken || num_cand_less<0)
      break;

    // float ratio = ((float)num_cand_less)/(float)num_cand_taken;
    float ratio = (float)num_cand_taken/((float)num_cand_less);
    int diff = std::log(ratio)/std::log(4);
    reg_level+=diff;
    reg_level=std::min(reg_level,5);
    reg_level=std::max(reg_level,1);
    // std::cout << "reg_level " << reg_level << std::endl;

  }

  delete img_magn;

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Candidates sampled: "+ std::to_string(count) + ", " + std::to_string(deltaTime)+" ms");

  return true;
}

void PointsHandler::showCandidates(){
  dso_->frame_current_->points_container_->showCandidates();
}
void PointsHandler::showProjectedCandidates(const std::string& name){
  dso_->frame_current_->points_container_->showProjectedCandidates(name);
}
void PointsHandler::showProjectedCandidates(){
  dso_->frame_current_->points_container_->showProjectedCandidates();
}
void PointsHandler::showActivePoints(){
  dso_->frame_current_->points_container_->showActivePoints();
}

void PointsHandler::showProjectedActivePoints( int i){
  dso_->frame_current_->points_container_->showProjectedActivePoints(i);
}
void PointsHandler::showProjectedActivePoints(const std::string& name, int i){
  dso_->frame_current_->points_container_->showProjectedActivePoints(name, i);
}

void PointsHandler::projectCandidatesOnLastFrame(){

  for (CandidateProjected* cand_proj : dso_->frame_current_->points_container_->candidates_projected_ )
    delete cand_proj;

  dso_->frame_current_->points_container_->candidates_projected_.clear();

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    projectCandidates(keyframe, dso_->frame_current_ );
  }
}

void PointsHandler::selfProjectCandidatesOnLastFrame(){
  projectCandidates(dso_->frame_current_, dso_->frame_current_ );
}

void PointsHandler::projectActivePointsOnLastFrame(){

  for (ActivePointProjected* active_pt_proj : dso_->frame_current_->points_container_->active_points_projected_ )
    delete active_pt_proj;

  dso_->frame_current_->points_container_->active_points_projected_.clear();

  // iterate through keyframes with active points
  for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->frames_with_active_pts_[i];
    projectActivePoints(keyframe, dso_->frame_current_ );
  }
}


void PointsHandler::projectCandidates(CameraForMapping* cam_r, CameraForMapping* cam_m ){
  std::shared_ptr<CamCouple> cam_couple = std::make_shared<CamCouple>(cam_r, cam_m);

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
  std::shared_ptr<CamCouple> cam_couple(new CamCouple(cam_r, cam_m) );

  // iterate through active points
  for(ActivePoint* active_pt : (cam_r->points_container_->active_points_)){
    if(active_pt->invdepth_==-1)
      continue;
    ActivePointProjected* active_pt_proj = (new ActivePointProjected(active_pt, cam_couple ));
    if( cam_m->image_intensity_->pixelInRange(active_pt_proj->pixel_) ){
      cam_m->points_container_->active_points_projected_.push_back(active_pt_proj);
    }
  }
}

void PointsHandler::trackCandidatesReverse(bool groundtruth){

  double t_start=getTime();

  sharedCoutDebug("   - Candidates reverse tracking: ");

  CameraForMapping* last_keyframe = dso_->cameras_container_->getLastActiveKeyframe();

  int counter = 0;
  // iterate through keyframes (except last)
  for (int i=dso_->cameras_container_->keyframes_active_.size()-2; i>=0; i--){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];


    if(groundtruth){
      trackCandidatesGroundtruth(keyframe);
    }
    else{

      n_cands_removed_=0;
      n_cands_to_track_=0;
      n_cands_var_too_high_=0;
      n_cands_repeptitive_=0;
      n_cands_no_min_=0;
      n_cands_tracked_=0;
      n_cands_ep_too_short_=0;
      n_cands_no_clear_min_=0;
      n_cands_updated_=0;

      trackCandidates(last_keyframe, keyframe, counter<2);
      // trackCandidates(keyframe, last_keyframe);

      sharedCoutDebug("       - Keyframe: "+ keyframe->name_ );
      sharedCoutDebug("           - total: " + std::to_string(n_cands_to_track_));
      sharedCoutDebug("           - tracked: " + std::to_string(n_cands_tracked_));
      sharedCoutDebug("               - updated: " + std::to_string(n_cands_updated_));
      sharedCoutDebug("               - not_updated: " + std::to_string(n_cands_ep_too_short_));
      sharedCoutDebug("           - removed: " + std::to_string(n_cands_removed_));
      sharedCoutDebug("               - repetitive: " + std::to_string(n_cands_repeptitive_));
      // sharedCoutDebug("               - ep segment too short: " + std::to_string(n_cands_ep_too_short_));
      sharedCoutDebug("               - no clear min: " + std::to_string(n_cands_no_clear_min_));
      sharedCoutDebug("               - var too high: " + std::to_string(n_cands_var_too_high_));
      sharedCoutDebug("               - no min: " + std::to_string(n_cands_no_min_));
    }

    counter++;
  }
}

void PointsHandler::removeOcclusionsInLastKFGrountruth(){
  projectActivePointsOnLastFrame();


  // iterate through active points projected
  for(int i=dso_->frame_current_->points_container_->active_points_projected_.size()-1; i>=0; i--){
    ActivePointProjected* active_pt_proj = dso_->frame_current_->points_container_->active_points_projected_[i];

    // take invdepth gt
    float invdepth_val = dso_->frame_current_->grountruth_camera_->invdepth_map_->evalPixel(active_pt_proj->pixel_);
    float invdepth_gt = invdepth_val/dso_->frame_current_->cam_parameters_->min_depth;

    // take current invdepth
    float invdepth_pred = active_pt_proj->invdepth_;

    if (1/invdepth_pred > (1/invdepth_gt)+0.2)
      active_pt_proj->active_pt_->remove();

  }
}


void PointsHandler::trackCandidates(bool groundtruth){

  double t_start=getTime();

  sharedCoutDebug("   - Candidates tracking: ");

  CameraForMapping* last_keyframe = dso_->cameras_container_->getLastActiveKeyframe();

  // iterate through keyframes (except last)
  for (int i=dso_->cameras_container_->keyframes_active_.size()-2; i>=0; i--){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];


    if(groundtruth){
      trackCandidatesGroundtruth(keyframe);
    }
    else{

      n_cands_removed_=0;
      n_cands_to_track_=0;
      n_cands_var_too_high_=0;
      n_cands_repeptitive_=0;
      n_cands_no_min_=0;
      n_cands_tracked_=0;
      n_cands_ep_too_short_=0;
      n_cands_no_clear_min_=0;
      n_cands_updated_=0;

      // trackCandidates(last_keyframe, keyframe);
      trackCandidates(keyframe, last_keyframe);

      sharedCoutDebug("       - Keyframe: "+ keyframe->name_ );
      sharedCoutDebug("           - total: " + std::to_string(n_cands_to_track_));
      sharedCoutDebug("           - tracked: " + std::to_string(n_cands_tracked_));
      sharedCoutDebug("               - updated: " + std::to_string(n_cands_updated_));
      sharedCoutDebug("               - not_updated: " + std::to_string(n_cands_ep_too_short_));
      sharedCoutDebug("           - removed: " + std::to_string(n_cands_removed_));
      sharedCoutDebug("               - repetitive: " + std::to_string(n_cands_repeptitive_));
      // sharedCoutDebug("               - ep segment too short: " + std::to_string(n_cands_ep_too_short_));
      sharedCoutDebug("               - no clear min: " + std::to_string(n_cands_no_clear_min_));
      sharedCoutDebug("               - var too high: " + std::to_string(n_cands_var_too_high_));
      sharedCoutDebug("               - no min: " + std::to_string(n_cands_no_min_));
    }
  }

  double t_end=getTime();
  sharedCoutDebug("   - Candidates tracked: " + std::to_string((int)(t_end-t_start)) + " ms");




}

void PointsHandler::trackCandidatesGroundtruth(CameraForMapping* keyframe){

  // iterate through candidates
  for (int i=0; i<keyframe->points_container_->candidates_.size(); i++){
    Candidate* cand = keyframe->points_container_->candidates_.at(i);
    cand->setInvdepthGroundtruth();
  }

}

void PointsHandler::trackCandidates(CameraForMapping* kf1, CameraForMapping* kf2, bool remove){

  std::shared_ptr<CamCouple> cam_couple =  std::make_shared<CamCouple>(kf1,kf2);

  n_cands_to_track_+= kf1->points_container_->candidates_.size();
  // iterate through candidates
  for (int i=kf1->points_container_->candidates_.size()-1; i>=0; i--){
    Candidate* cand = kf1->points_container_->candidates_.at(i);

    if( !trackCandidate(cand, cam_couple) && remove ){
      cand->remove();
      n_cands_removed_++;
    }

  }

}

bool PointsHandler::trackCandidate(Candidate* cand, std::shared_ptr<CamCouple> cam_couple){
  // get uv of min and max depth
  Eigen::Vector2f uv_min, uv_max;
  // cam_couple->getUv(cand->uv_.x(),cand->uv_.y(),cand->depth_min_,uv_min.x(),uv_min.y());
  // cam_couple->getUv(cand->uv_.x(),cand->uv_.y(),cand->depth_max_,uv_max.x(),uv_max.y());
  cam_couple->reprojection(cand->uv_,cand->depth_min_,uv_min);
  cam_couple->reprojection(cand->uv_,cand->depth_max_,uv_max);

  // check if dp/dinvdpth is too small
  float der = (uv_min-uv_max).norm()/((1.0/cand->depth_min_)-(1.0/cand->depth_max_));
  // if(der<der_threshold){
  //   n_cands_var_too_high_++;
  //   return false;
  // }

  // create epipolar line
  EpipolarLine ep_segment( cam_couple->cam_m_, uv_min, uv_max, cand->level_) ;


  // search pixel in epipolar line
  CandTracker CandTracker(ep_segment, cand, cam_couple );
  int min_found = CandTracker.searchMin();

  switch (min_found){
    case 0:{
      n_cands_tracked_++;
      n_cands_updated_++;
      if (debug_mapping_match && dso_->frame_current_idx_>=debug_start_frame){
        cand->showCandidate();
        CandTracker.ep_segment_.showEpipolarWithMin(CandTracker.pixel_, red, cand->level_, 2);
      }
      return true;
    }
    case 1:{
      n_cands_repeptitive_++;
      return false;
    }
    case 2:{
      // cand->cost_threshold_*1.1;
      n_cands_no_min_++;
      return false;
    }
    case 3:{
      n_cands_no_clear_min_++;
      return false;
    }
    case 4:{
      n_cands_var_too_high_++;
      return false;
    }
    case 5:{
      n_cands_ep_too_short_++;
      n_cands_tracked_++;
      return true;
    }
  }
  return false;
}

float CandTracker::getStandardDeviation( ){


  // // GEOMETRIC DISPARITY ERROR
  // float g_dot_l; // squared scalar product between direction of gradient and ep_line -> |g| |l| cos(a)
  //                // since |g|, |l| =1 -> cos(angle between g and l)
  // float angle_g = phase_m;
  // float angle_l =ep_segment_.slope2angle();
  // if (angle_l<0)
  //   angle_l+=2*PI;
  // float a = radiansSub(angle_g,angle_l);
  // float c_a = cos(a);
  // g_dot_l=abs(c_a);
  // // standard deviation epipolar line (fixed)
  // float sd_epline_geo = cand_->cam_->cam_parameters_->pixel_width/4;
  // // standard deviation disparity
  // float sd_disparity_geo = sd_epline_geo/(g_dot_l+0.1);
  //
  //
  // // PHOTOMETRIC ERROR
  // // gradient on epline direction
  // float magn_g = magn_m;
  // float g_p = g_dot_l*magn_g;
  // // standard deviation img noise
  // // float sd_img_noise = sd_img_noise;
  // float sd_img_noise = cand_->cam_->cam_parameters_->pixel_width/400;
  // // standard deviation photometric
  // float sd_disparity_photometric = abs(sd_img_noise/(g_p+0.01));
  //
  //
  // // standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric+sd_epline_sampling);
  // float standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric);

  float standard_deviation = cand_->cam_->cam_parameters_->pixel_width;

  return standard_deviation;
}


bool CandTracker::updateCand(int max_pxls_inliers){

  float coord;
  if(ep_segment_.u_or_v)
    coord=uv_.x();
  else
    coord=uv_.y();

  // update depth / invdepth
  float depth;
  bool valid_d = cam_couple_->getD1( cand_->uv_.x(), cand_->uv_.y(), depth, coord, ep_segment_.u_or_v );
  cand_->invdepth_=1./depth;

  // get standard deviation
  // float standard_deviation = getStandardDeviation();
  // float standard_deviation = cand_->cam_->cam_parameters_->pixel_width*max_pxls_inliers;
  float standard_deviation = cand_->cam_->cam_parameters_->pixel_width;

  // update bounds
  float bound_min, bound_max;
  int sign = pow(-1,(ep_segment_.start>ep_segment_.end));
  float coord_min = coord-sign*standard_deviation;
  float coord_max = coord+sign*standard_deviation;
  bool valid_min = cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), cand_->depth_min_, coord_min, ep_segment_.u_or_v);
  bool valid_max = cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), cand_->depth_max_, coord_max, ep_segment_.u_or_v);

  if(!valid_d || !valid_min || !valid_max)
    return false;

  cand_->invdepth_var_= pow(((1./cand_->depth_min_)-(1./cand_->depth_max_))*0.5,2)*1000+0.1;
  // cand_->invdepth_var_= ((1./cand_->depth_min_)-(1./cand_->depth_max_))*500;

  // cand_->invdepth_var_= max_pxls_inliers*0.2;
  // cand_->invdepth_var_= 1;

  if(cand_->invdepth_var_>var_threshold){
    return false;
  }
  return true;
}

int CandTracker::searchMin( ){
  bool min_segment_reached = false;
  bool min_segment_leaved = false;
  float cost_min = FLT_MAX;
  int n_pxls_inliers = 0;

  // iterate through uvs
  for(int i=0; i<ep_segment_.uvs.size(); i++){
    Eigen::Vector2f uv_m = ep_segment_.uvs[i];
    pxl pixel_m = cam_couple_->cam_m_->uv2pixelCoords( uv_m, cand_->level_);

    if(!cam_couple_->cam_m_->pyramid_->getC(cand_->level_)->pixelInRange(pixel_m))
      continue;

    n_valid_uvs_++;

    float cost_magn = getCostMagn(pixel_m);
    float cost_phase=0;
    bool valid = getPhaseCostContribute(pixel_m, uv_m, cost_phase);
    if(!valid)
      continue;
      // return false;

    float cost=cost_magn+cost_phase;
    // float cost=cost_magn;

    if(cost>cand_->cost_threshold_mapping_){
    // if(cost>cost_threshold){
      if(min_segment_reached){
        min_segment_leaved=true;
      }

    }
    else{

      // repetitive
      if(min_segment_leaved){
        return 1;
      }

      n_pxls_inliers++;
      if (n_pxls_inliers>max_pxls_inliers)
        return 3;

      min_segment_reached=true;


      if(cost<cost_min){
        cost_min=cost;
        uv_=uv_m;
        pixel_=pixel_m;
      }

    }
  }

  if (n_valid_uvs_<=3){
    return 5;
  }

  // no mins
  if(!min_segment_reached)
    return 2;

  if(updateCand(max_pxls_inliers))
    return 0;
  else
    return 4;
}

float CandTracker::getCostMagn(pxl& pixel){

  float cost_c = getCostIntensity(pixel);
  float cost_magn_cd = getCostGradient(pixel);

  return intensity_coeff_mapping*cost_c+gradient_coeff_mapping*cost_magn_cd;
}

float CandTracker::getCostIntensity(pxl& pixel_m){
  pixelIntensity c_m = cam_couple_->cam_m_->pyramid_->getC(cand_->level_)->evalPixelBilinear(pixel_m);
  pixelIntensity c_r = cand_->c_;
  // return abs(c_r-c_m);

  float error = cam_couple_->getErrorIntensity(c_r, c_m);
  return abs(error);
}

float CandTracker::getCostGradient(pxl& pixel_m){
  magn_m = cam_couple_->cam_m_->pyramid_->getMagn(cand_->level_)->evalPixelBilinear(pixel_m);
  pixelIntensity magn_cd_r = cand_->magn_cd_;
  // return abs(magn_cd_r-magn_m);

  float error = cam_couple_->getErrorGradient(magn_cd_r, magn_m);
  return abs(error);

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

  bool valid = cam_couple_->getD1(cand_->uv_.x(), cand_->uv_.y(), d1, coord, ep_segment_.u_or_v);
  if (!valid)
    return false;

  Eigen::Vector2f tip_m, direction_m;

  // cam_couple_->getUv(tip_to_project.x(), tip_to_project.y(), d1, tip_m.x(), tip_m.y() );
  float depth;
  cam_couple_->reprojection(tip_to_project, d1, tip_m, depth );

  direction_m=tip_m-uv_m;

  // std::cout << "\n1:\n" << direction_m << " 2:\n " << grad_direction_r << std::endl;

  float phase_m_hat = std::atan2(direction_m.y(),direction_m.x());
  if(!std::isfinite(phase_m_hat))
    return false;

  if (phase_m_hat<0)
    phase_m_hat+=2*PI;
  if (phase_m<0)
    phase_m+=2*PI;

  if(abs(phase_m)>=5 || abs(phase_m_hat)>=5){
    return false;
  }
  // if(abs(phase_m_hat)>=10){
  // }
  // if( (phase_far < phase_m && phase_m < phase_close) || (phase_far > phase_m && phase_m > phase_close) ){
  assert(abs(phase_m_hat)<10);
  assert(abs(phase_m)<10);

  phase_cost= abs(radiansSub(phase_m_hat,phase_m))*phase_coeff;
  return true;
}
