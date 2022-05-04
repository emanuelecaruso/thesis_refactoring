#include "BundleAdj.h"
#include "LinSystemBA.h"
#include "utils.h"
#include "dso.h"

bool BundleAdj::getMeasurements(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector){

  int n_valid = 0;
  int n_occlusions = 0;

  // get only errors
  // iterate through all active keyframes
  for( int j=0; j<dso_->cameras_container_->keyframes_active_.size() ; j++){

    // avoid self projections
    if (i==j)
      continue;

    CamCouple* cam_couple = cam_couple_container_->get(i,j);
    assert(cam_couple->cam_r_==active_point->cam_);
    assert(cam_couple->cam_m_==dso_->cameras_container_->keyframes_active_[j]);

    MeasBA* measurement = new MeasBA(active_point, cam_couple, dso_->parameters_->chi_occlusion_threshold);
    if(measurement->occlusion_){
      n_occlusions++;
    }
    else if(measurement->valid_){
      measurement_vector->push_back(measurement);
      n_valid++;
    }

  }

  // evaluate occlusion ratio
  float occlusion_valid_ratio = (float)n_occlusions/(float)n_valid;
  // evaluate non valid proj ratio
  float valid_ratio = (float)n_valid/((float)dso_->cameras_container_->keyframes_active_.size()-1);

  // if(occlusion_valid_ratio>0.25){
  if(occlusion_valid_ratio>dso_->parameters_->occlusion_valid_ratio_thresh ||
     valid_ratio<dso_->parameters_->valid_ratio_thresh){
    // clear vector
    for( MeasBA* measurement : *measurement_vector ){
      delete measurement;
    }
    return false;
  }


  // load jacobians
  // iterate through all active keyframes
  for( MeasBA* measurement : *measurement_vector){
    measurement->loadJacobians( active_point );

  }
  return true;
}

void BundleAdj::setCamData(){
  int count =0;

  // iterate through keyframes
  for(int i=0; i<dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    if(!keyframe->fixed_){
      keyframe->data_for_ba_->c_idx_=count;
      count++;
    }
  }
}

void BundleAdj::marginalizePoint(ActivePoint* active_point, CamCoupleContainer* cam_couple_container){
  // remove candidate from vector
  std::vector<ActivePoint*>& v = active_point->cam_->points_container_->active_points_;
  int v_size = v.size();
  v.erase(std::remove(v.begin(), v.end(), active_point), v.end());
  assert(v_size==v.size()+1);

  MarginalizedPoint* marg_pt = new MarginalizedPoint(active_point);

  // push marginalized point
  active_point->cam_->points_container_->marginalized_points_.push_back(marg_pt);

  // crete prior
  // iterate through all active keyframes
  for (int i=0; i<cam_couple_container->cam_couple_mat_[0].size(); i++){
    CamCouple* cam_couple = cam_couple_container->get(0,i);
    Prior* prior = new Prior(active_point, cam_couple);
    if(!prior->valid_){
      delete prior;
      continue;
    }
    else{
      cam_couple->cam_m_->data_for_ba_->priors_.push_back(prior);
    }
  }

  // delete active point
  delete active_point;

}


void BundleAdj::marginalizePointsAndKeyframes(){



  CameraForMapping* curr_frame = dso_->frame_current_;

  // iterate through keyframes to be marginalized
  for (CameraForMapping* keyframe_to_marg : dso_->cameras_container_->keyframes_to_marginalize_){

    // create cam couple vector
    CamCoupleContainer* cam_couple_container = new CamCoupleContainer(dso_,keyframe_to_marg);

    // marginalize also all active points
    for(int i=keyframe_to_marg->points_container_->active_points_.size()-1; i>=0; i--){
      ActivePoint* active_pt = keyframe_to_marg->points_container_->active_points_[i];
      marginalizePoint(active_pt, cam_couple_container);
    }

    delete cam_couple_container;
  }

  // marginalize points not in last cam
  // iterate through all keyframe (except the last)
  for (int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    // CamCouple* cam_couple_curr_frame = new CamCouple(keyframe,curr_frame);

    CamCoupleContainer* cam_couple_container = new CamCoupleContainer(dso_,keyframe);

    // iterate along all active points
    for(int i=keyframe->points_container_->active_points_.size()-1; i>=0; i--){
      ActivePoint* active_pt = keyframe->points_container_->active_points_[i];

      Eigen::Vector2f uv_curr;
      cam_couple_container->get(0,dso_->cameras_container_->keyframes_active_.size()-1)->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_curr.x(),uv_curr.y() );
      bool uv_in_range = keyframe->uvInRange(uv_curr);
      if(!uv_in_range){
        marginalizePoint(active_pt, cam_couple_container);
      }
    }

    delete cam_couple_container;
    // delete cam_couple_curr_frame;
  }

}

void BundleAdj::optimize(){

  double t_start=getTime();

  // prepare cams data for bundle adjustment
  // setCamData();
  cam_couple_container_->init();

  // create linear system
  LinSysBA lin_sys_tracking(dso_);

  // iterations of bundle adjustment
  for(int iteration=0; iteration<dso_->parameters_->max_iterations_ba; iteration++){

    int num_points = 0;

    std::vector<std::vector<MeasBA*>*> measurement_vec_vec;

    // iterate through active keyframes (except last)
    for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
      CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[i];

      // for( ActivePoint* active_pt : cam_r->points_container_->active_points_ ){
      for( int j=cam_r->points_container_->active_points_.size()-1; j>=0; j-- ){
        ActivePoint* active_pt = cam_r->points_container_->active_points_[j];

        std::vector<MeasBA*>* measurement_vector = new std::vector<MeasBA*>;
        bool measurements_are_valid = getMeasurements(active_pt, i, measurement_vector);
        if(measurements_are_valid){
          active_pt->p_idx_=num_points;
          num_points++;
          measurement_vec_vec.push_back(measurement_vector);
        }
        else {
          active_pt->remove();
        }

      }
    }

    lin_sys_tracking.reinitWithNewPoints(num_points);

    lin_sys_tracking.buildLinearSystem(measurement_vec_vec);
    lin_sys_tracking.updateState();

    cam_couple_container_->init();

    if(dso_->parameters_->debug_optimization){
      dso_->spectator_->renderState();
      dso_->spectator_->showSpectator();
    }

  }

  double t_end=getTime();
  int deltaTime = (t_end-t_start);
  sharedCoutDebug("   - Bundle adjustment: " + std::to_string(deltaTime) + " ms");


}
