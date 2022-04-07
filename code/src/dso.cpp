#include "dso.h"
#include "utils.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"

void Dso::startSequential(){
  std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
  update_cameras_thread_.detach();

  while(true){

    loadFrameCurrent();

    if(first_frame_to_set_){
      setFirstKeyframe();
    }
    else if(to_initialize_){
      initialize();
    }
    else{

    }

  }
}

void Dso::loadFrameCurrent(){

  std::cout << frame_current_idx_ << " " << int(cameras_container_->frames_.size())-1 << "\n";
  if(frame_current_idx_==int(cameras_container_->frames_.size())-1){
    waitForNewFrame();
  }

  if(parameters_->get_current_frame){
    frame_current_idx_=cameras_container_->frames_.size();
    frame_current_=cameras_container_->frames_.back();
  }
  else{
    frame_current_idx_++;
    frame_current_=cameras_container_->frames_[frame_current_idx_];
  }
}

void Dso::setFirstKeyframe(){

  tracker_->trackCam(true); //groundtruth
  keyframe_handler_->addKeyframe(true);  // add fixed keyframe
  initializer_->extractCorners();
  if(parameters_->debug_initialization){
    initializer_->showCornersTrackCurr();
  }
  points_handler_->sampleCandidates();
  if(parameters_->debug_mapping){
    points_handler_->showCandidates();
  }
  first_frame_to_set_=false;
}

void Dso::initialize(){
  initializer_->trackCornersLK();

  // if a good pose is found ...
  if(initializer_->findPose()){
    sharedCoutDebug("   - Pose found");
    if(parameters_->debug_initialization){
      initializer_->showCornersTrackCurr();
    }

    // ... add last keyframe
    keyframe_handler_->addKeyframe(true);

    // // start initializing the model
    // mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
    //
    // bundle_adj_->projectActivePoints_prepMarg(0);
    // bundle_adj_->activateNewPoints();
    // bundle_adj_->collectCoarseActivePoints();
    //
    // keyframe_handler_->prepareDataForBA();
    // int test_single = bundle_adj_->test_single_;
    // bundle_adj_->test_single_=TEST_ONLY_POINTS;
    // bundle_adj_->optimize();
    // bundle_adj_->test_single_=test_single;
    //
    // mapper_->selectNewCandidates();
    //
    // initialization_done=true;
    //
    // double t_end=getTime();
    // int deltaTime=(t_end-t_start);
    // sharedCoutDebug("   - INITIALIZATION of frame "+std::to_string(frame_current_)+", computation time: "+ std::to_string(deltaTime)+" ms");
    // if(initialization_done){
    //   sharedCoutDebug("\nINITIALIZATION ENDED");
    // }

  }

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

    std::shared_ptr<Camera> cam = environment_->camera_vector_->at(counter);
    cameras_container_->addFrame(cam);

    // locker.unlock();
    // std::cout << "CAM UPDATED" << std::endl;
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
  sharedCout("\nVideo stream ended");

}
