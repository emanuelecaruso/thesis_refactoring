#include "dso.h"
#include "utils.h"

void Dso::startSequential(){
  std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
  update_cameras_thread_.detach();

  while(true){

    loadFrameCurrent();

    if(first_frame_to_set_){
      setFirstKeyframe();
    }
    else if(to_initialize_){

    }
    else{

    }

  }
}

void Dso::loadFrameCurrent(){

  std::cout << frame_current_idx_ << " " << int(cameras_container_.frames_.size())-1 << "\n";
  if(frame_current_idx_==int(cameras_container_.frames_.size())-1){
    waitForNewFrame();
  }

  if(parameters_->get_current_frame){
    frame_current_idx_=cameras_container_.frames_.size();
    frame_current_=cameras_container_.frames_.back();
  }
  else{
    frame_current_idx_++;
    frame_current_=cameras_container_.frames_[frame_current_idx_];
  }
}

void Dso::setFirstKeyframe(){

  tracker_.trackCam(true); //groundtruth
  keyframe_handler_.addKeyframe(true);  // add fixed keyframe
  initializer_.extractCorners();
  if(parameters_->debug_initialization){
    initializer_.showCornersTrackCurr();
  }
  // mapper_->selectNewCandidates();
  first_frame_to_set_=false;
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
    cameras_container_.addFrame(cam);

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
