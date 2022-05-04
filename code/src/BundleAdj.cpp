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
  if(occlusion_valid_ratio>0.5){
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

  }

  double t_end=getTime();
  int deltaTime = (t_end-t_start);
  sharedCoutDebug("   - Bundle adjustment: " + std::to_string(deltaTime) + " ms");


}
