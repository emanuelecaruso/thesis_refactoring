#include "dso.h"
#include "utils.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"


PoseNormError Dso::getTotalPosesNormError(){
  PoseNormError poses_norm_error_tot;
  int count = 0;
  for (CameraForMapping* cam : cameras_container_->frames_){
    if (cam->keyframe_){
      PoseNormError cam_pose_norm_error = cam->getPoseNormError();
      poses_norm_error_tot+=cam_pose_norm_error;
      count++;
    }
  }
  poses_norm_error_tot/=count;
  return poses_norm_error_tot;
}

void Dso::startSequential(){
  std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
  update_cameras_thread_.detach();

  while(true){

    if(!loadFrameCurrent())
      break;

    if(first_frame_to_set_){
      setFirstKeyframe();
    }
    else if(to_initialize_){
      initialize();
    }
    else{
      doDso();
    }
  }

  PoseNormError pose_norm_error = getTotalPosesNormError();
  pose_norm_error.print();
}

bool Dso::loadFrameCurrent(){

  // sharedCoutDebug("\nFrame "+std::to_string(frame_current_idx_)+" ("+frame_current_->name_+") , frame delay: "+std::to_string(frame_delay));
  if(frame_current_idx_>=end_frame-1)
    return false;

  if(frame_current_idx_==int(cameras_container_->frames_.size())-1){
    waitForNewFrame();
  }

  if(get_current_frame){
    frame_current_idx_=cameras_container_->frames_.size();
    frame_current_=cameras_container_->frames_.back();
  }
  else{
    frame_current_idx_++;
    frame_current_=cameras_container_->frames_[frame_current_idx_];
  }

  sharedCoutDebug("\nFrame "+std::to_string(frame_current_idx_)+", delay: "+std::to_string(cameras_container_->frames_.size()-frame_current_idx_));
  return true;
}

void Dso::setFirstKeyframe(){

  tracker_->trackCam(true); // fix first frame to groundtruth pose
  keyframe_handler_->addKeyframe(true);  // add fixed keyframe
  initializer_->extractCorners(); // extract corners from image
  if(debug_initialization){
    initializer_->showCornersTrackRef();
  }
  points_handler_->sampleCandidates(); // sample candidates as high gradient points
  if(debug_mapping){
    points_handler_->showCandidates();
  }
  first_frame_to_set_=false;
}

void Dso::initialize(){

  if(take_gt_initialization){
    tracker_->trackCam(true); // fix first frame to groundtruth pose
  }
  else{

    // initializer_->trackCornersLK(); // track corners in subsequent image

    // if a good pose is found ...
    if(initializer_->findPose()){

      if(debug_initialization){
        initializer_->showCornersTrackCurr();
      }

      keyframe_handler_->addKeyframe(true);  // add fixed keyframe
      if(use_spectator){
        spectator_->renderState();
        spectator_->showSpectator(0);
      }
      points_handler_->trackCandidates(take_gt_points); // track existing candidates

      // project candidates and active points on last frame
      candidates_activator_->activateCandidates();
      points_handler_->sampleCandidates(); // sample candidates as high gradient points

      bundle_adj_->optimize(true);

      to_initialize_=false;

      if(debug_mapping){

        // cameras_container_->keyframes_active_[0]->points_container_->showCandidates();
        // points_handler_->sampleCandidates();
        // points_handler_->showCandidates();
        // points_handler_->projectActivePointsOnLastFrame();

        points_handler_->projectCandidatesOnLastFrame();
        points_handler_->projectActivePointsOnLastFrame();
        points_handler_->showProjectedCandidates( "cands proj");
        points_handler_->showProjectedActivePoints(" act pts proj");
      }
    }

  }
}


bool Dso::doDso(){

  // track cam
  tracker_->trackCam(take_gt_poses);

  // add keyframe
  bool kf_added = keyframe_handler_->addKeyframe(false); // add keyframe
  if(kf_added){

    // track existing candidates
    points_handler_->trackCandidates(take_gt_points);

    points_handler_->projectCandidatesOnLastFrame();
    // points_handler_->showProjectedCandidates( "cands proj 0/1");

    // activate points
    candidates_activator_->activateCandidates();

    // sample new candidates
    points_handler_->sampleCandidates(); // sample candidates as high gradient points

    // debug mapping
    if(debug_mapping && frame_current_idx_>=debug_start_frame){
      // cameras_container_->keyframes_active_[0]->points_container_->showCoarseActivePoints(2);
      // cameras_container_->keyframes_active_[0]->points_container_->showCandidates();
      // points_handler_->showCandidates();
      points_handler_->projectCandidatesOnLastFrame();
      points_handler_->projectActivePointsOnLastFrame();
      points_handler_->showProjectedCandidates( "cands proj");
      points_handler_->showProjectedActivePoints(" act pts proj");
    }
  }

  if (cameras_container_->keyframes_active_.size()>2){

    // marginalize points not in last camera
    bundle_adj_->marginalize();
    // bundle adjustment optimization
    bundle_adj_->optimize();
  }


  if(use_spectator){

    // for( int i = 0; i<cameras_container_->keyframes_active_.size()-1; i++){
    //   frame_current_->points_container_->active_points_projected_.clear();
    //   CameraForMapping* keyframe = cameras_container_->keyframes_active_[i];
    //   points_handler_->projectActivePoints( keyframe , frame_current_ );
    //   points_handler_->showProjectedActivePoints("last kf "+std::to_string(i),0);
    // }

    points_handler_->showProjectedActivePoints("last kf ",1);
    spectator_->renderState();
    spectator_->showSpectator(1);
  }

  return true;

}

void Dso::waitForNewFrame(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  frame_updated_.wait(locker);
  locker.unlock();
}

void Dso::updateCamerasFromEnvironment(){

  assert(environment_->camera_vector_->size()!=0);

  float fps=environment_->fps_;
  int counter=0;

  while(counter< environment_->camera_vector_->size()){

    // std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    // sharedCout("\nFrame: "+ std::to_string(counter));

    Camera* cam = environment_->camera_vector_->at(counter);
    cameras_container_->addFrame(cam);

    // locker.unlock();
    frame_updated_.notify_all();

    double t_end=getTime();

    int deltaTime=(t_end-t_start);
    // sharedCoutDebug("\nAdd Frame "+std::to_string(counter)+", computation time: "+ std::to_string(deltaTime)+" ms");
    long int waitDelay=deltaTime*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    if(time_to_wait>0)
      std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
    // else
      // sharedCoutDebug("Delay accumulated! : "+ std::to_string(-time_to_wait)+" ms");


    counter++;

  }
  // update_cameras_thread_finished_=true;
  // frame_updated_.notify_all();
  sharedCoutDebug("\nVideo stream ended");

}
