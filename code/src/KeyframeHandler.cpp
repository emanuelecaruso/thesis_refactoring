#include "KeyframeHandler.h"
#include "CameraForMapping.h"
#include "dso.h"

bool KeyframeHandler::addKeyframe(bool fixed){
  dso_->frame_current_->fixed_=fixed;

  if (fixed){
    dso_->cameras_container_->addActiveKeyframe(dso_->frame_current_);
    sharedCoutDebug("   - Keyframe added (fixed)");
    marginalizeKeyframe();
    return true;
  }
  else{
    // get optical flow distance
    float flow_dist = getFlowDist();

    // eval optical flow distance
    if(flow_dist>dso_->parameters_->flow_dist_threshold){
      dso_->cameras_container_->addActiveKeyframe(dso_->frame_current_);
      sharedCoutDebug("   - Keyframe added");
      marginalizeKeyframe();
      return true;
    }
    else{
      sharedCoutDebug("   - Keyframe not added");
      return false;
    }

  }


}

float KeyframeHandler::getPercentuageMarg(CameraForMapping* keyframe){
  int num_active_pts = keyframe->points_container_->active_points_.size();
  int num_non_active_pts = keyframe->points_container_->marginalized_points_.size()+keyframe->points_container_->n_active_points_removed_;
  int num_tot_pts = num_active_pts+num_non_active_pts;

  float percentage_marg = 1;
  if(num_tot_pts>0){
    percentage_marg= (((float)num_active_pts)/((float)(num_tot_pts)));
  }
  return percentage_marg;
}

void KeyframeHandler::marginalize(CameraForMapping* keyframe){
  dso_->cameras_container_->addKeyframeToMarginalize(keyframe);
  sharedCoutDebug("   - "+keyframe->name_+" to marginalize");


}


float KeyframeHandler::getScore(CameraForMapping* keyframe){

  CameraForMapping* last_kf = dso_->cameras_container_->keyframes_active_.back();

  Eigen::Vector3f pos_diff_last_kf = (keyframe->frame_camera_wrt_world_->translation())-(last_kf->frame_camera_wrt_world_->translation());
  float d1 = sqrt(pos_diff_last_kf.norm());
  float sum = 0;
  // iterate through all keyframe (except the last)
  for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1; i++){
  // for (int i=0; i<dso_->keyframe_vector_->size()-2; i++){
    CameraForMapping* keyframe_ = dso_->cameras_container_->keyframes_active_[i];

    if(keyframe_==keyframe)
      continue;

    Eigen::Vector3f pos_diff = (keyframe->frame_camera_wrt_world_->translation())-(keyframe_->frame_camera_wrt_world_->translation());
    sum += 1.0/(pos_diff.norm()+0.00000001);

  }
  return d1*sum;


}

bool KeyframeHandler::marginalizeKeyframe(){

  bool percentage_marginalization = false;

  if( dso_->cameras_container_->keyframes_active_.size() <= dso_->parameters_->num_active_keyframes )
    return false;

  // iterate through all keyframe (except the last two)
  for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-2; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];

    float percentage_marg = getPercentuageMarg(keyframe);

    if(percentage_marg<dso_->parameters_->percentage_marg_pts_threshold){
      marginalize(keyframe);
      break;
      percentage_marginalization=true;
    }

  }

  if(!percentage_marginalization){

    CameraForMapping* keyframe_to_marg;
    float max_score = 0;
    // iterate through all keyframe (except the last)
    for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1; i++){
      CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];

      float score = getScore(keyframe);
      if (score>max_score){
        max_score=score;
        keyframe_to_marg=keyframe;
      }
    }
    marginalize(keyframe_to_marg);
  }

  // fix first 2 keyframes
  dso_->cameras_container_->keyframes_active_[0]->fixed_=true;
  dso_->cameras_container_->keyframes_active_[1]->fixed_=true;

  return true;

}



float KeyframeHandler::getFlowDist(){
  // project active points in last two frames
  CameraForMapping* curr_frame = dso_->frame_current_;
  CameraForMapping* last_kf = dso_->cameras_container_->keyframes_active_.back();

  float sum_dist = 0;
  int num_dists = 0;

  // iterate through all keyframe (except the last)
  for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    CamCouple* cam_couple_curr_frame = new CamCouple(keyframe,curr_frame);
    CamCouple* cam_couple_last_kf = new CamCouple(keyframe,last_kf);
    // iterate along all active points
    for (ActivePoint* active_pt : keyframe->points_container_->active_points_){

      Eigen::Vector2f uv_curr, uv_last;
      cam_couple_curr_frame->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_curr.x(),uv_curr.y() );
      cam_couple_last_kf->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_last.x(),uv_last.y() );

      float dist = getEuclideanDistance(uv_curr, uv_last);
      sum_dist+=dist;
      num_dists++;
    }

    delete cam_couple_curr_frame;
    delete cam_couple_last_kf;
  }

  float avg_dist = sum_dist/num_dists;

  return avg_dist;

}
