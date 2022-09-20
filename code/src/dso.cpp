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

  sharedCoutDebug("\n Start dso Sequential ");

  // double t_start=getTime();
  float fps=environment_->fps_;
  int counter=start_frame;
  while(counter< environment_->camera_vector_->size()){

    Camera* cam = environment_->camera_vector_->at(counter);
    cameras_container_->addFrame(cam);
    if(!loadFrameCurrent())
      break;

    sharedCoutDebug("\nAdd Frame "+std::to_string(counter));

    if(first_frame_to_set_){
      setFirstKeyframe();
    }
    else if(to_initialize_){
      initialize();
    }
    else{
      if(collect_invdepth_errs)
        testInvdepths();
      else{
        doDso();
      }
    }

    counter++;
    // double t_end=getTime();
    // double deltaTime=(t_end-t_start);
    // int frame_delay= (float)((deltaTime/1000)*fps)-counter;
    // sharedCoutDebug("\nFrame "+std::to_string(frame_current_idx_)+" ("+frame_current_->name_+") , frame delay: "+std::to_string(frame_delay));

  }
  video_spec_.release();
  video_proj_.release();
}


void Dso::startParallel(){
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
  if(debug_initialization && frame_current_idx_>=debug_start_frame){
    initializer_->showCornersTrackRef();
  }
  points_handler_->sampleCandidates(frame_current_); // sample candidates as high gradient points
  if(debug_mapping && frame_current_idx_>=debug_start_frame){
    points_handler_->showCandidates();
  }
  first_frame_to_set_=false;
  if(use_spectator){
    spectator_->renderState();
    spectator_->showSpectator(1);
  }
}

bool Dso::initialize(){

  if(take_gt_initialization){
    tracker_->trackCam(true); // fix first frame to groundtruth pose
  }
  else{
    // if a good pose is not found ...
    if(!initializer_->findPose())
      return false;
  }

  if(use_spectator){
    spectator_->renderState();
    spectator_->showSpectator(1);
  }

  keyframe_handler_->addKeyframe(true);  // add fixed keyframe

  points_handler_->sampleCandidates(frame_current_); // sample candidates as high gradient points



  if(reverse_tracking)
    points_handler_->trackCandidatesReverse(take_gt_points); // track existing candidates
  points_handler_->trackCandidates(take_gt_points); // track existing candidates

  // project candidates and active points on last frame
  candidates_activator_->activateCandidates();

  if(remove_occlusions_gt)
    points_handler_->removeOcclusionsInLastKFGrountruth();

  bundle_adj_->optimize(true);

  to_initialize_=false;

  if(debug_mapping && frame_current_idx_>=debug_start_frame){

    // cameras_container_->keyframes_active_[0]->points_container_->showCandidates();
    // points_handler_->sampleCandidates();
    // points_handler_->showCandidates();
    // points_handler_->projectActivePointsOnLastFrame();

    points_handler_->projectCandidatesOnLastFrame();
    points_handler_->projectActivePointsOnLastFrame();
    points_handler_->showProjectedCandidates( "cands proj");
    points_handler_->showProjectedActivePoints(" act pts proj");
  }

  return true;

}

float Dso::getMeanInvdepthErr(){
  float err_sum=0;
  for( float err : invdepth_errors)
    err_sum += err;

  return err_sum/invdepth_errors.size();
}

void Dso::plotInvdepthAccWithDer(){

  int size = 100;
  float stride = 0.3/size;
  float range = stride*10;
  float scale_var = 0.2;

  std::vector<float> means_x;
  std::vector<float> means_y;
  std::vector<float> vars_y;
  std::vector<float> u;
  std::vector<float> d;

  float idx = 0;
  for(int i=0; i<size; i++){

    float center = stride*i;
    float lb = std::fmax(0,center-range);
    float ub = std::fmin(gradients.back(),center+range);

    auto iter= std::upper_bound(gradients.begin(), gradients.end(), lb);
    idx = iter - gradients.begin();

    // std::cout << "start " << gradients[idx] << std::endl;

    int count = 0;
    float grad_sum = 0;
    float err_sum = 0;
    while(true){

      float grad = gradients[idx+count];
      float err = invdepth_errors[idx+count];
      if (grad > ub || idx+count>=gradients.size()-1){
        // std::cout << "end " << gradients[idx+count-1] << std::endl;
        break;
      }

      grad_sum+=grad;
      err_sum+=err;

      count++;

    }

    if (count==0){
      continue;
    }

    means_x.push_back(grad_sum/count);
    means_y.push_back(err_sum/count);

    float err_var_sum = 0;
    int count2 = 0;
    while(true){

      float grad = gradients[idx+count2];
      float err = invdepth_errors[idx+count2];
      if (grad > ub || idx+count2>=gradients.size()-1){
        break;
      }

      err_var_sum+=pow(invdepth_errors[idx+count2]-means_y[i],2);

      count2++;

    }
    vars_y.push_back((err_var_sum/count)*scale_var);

    u.push_back(means_y[i] + vars_y[i]);
    d.push_back(means_y[i] - vars_y[i]);


  }
  float a =0;
  matplotlibcpp::xlim(a, stride*size);
  matplotlibcpp::plot(means_x,means_y);

  // matplotlibcpp::fill_between(means_x, d, u)

  matplotlibcpp::plot(means_x,u,"r");
  matplotlibcpp::plot(means_x,d,"r");

  matplotlibcpp::show();

}


void Dso::testInvdepths(){

    // track cam
    tracker_->trackCam(true);

    // add keyframe
    bool kf_added = keyframe_handler_->addKeyframe(false); // add keyframe
    if(kf_added){

      // sample new candidates
      points_handler_->sampleCandidates(frame_current_); // sample candidates as high gradient points
      points_handler_->removeOcclusionsInLastKFGrountruthCands();


      points_handler_->collectInvdepths();

      points_handler_->projectCandidatesOnLastFrame();
      points_handler_->showProjectedCandidates( "cands proj");

    }

    if(!kf_added){
      frame_current_->cam_free_mem();
      delete frame_current_->points_container_;
    }
}



bool Dso::doDso(){

  // track cam
  tracker_->trackCam(take_gt_poses);

  // add keyframe
  bool kf_added = keyframe_handler_->addKeyframe(false); // add keyframe
  if(kf_added){

    // sample new candidates
    points_handler_->sampleCandidates(frame_current_); // sample candidates as high gradient points

    if(reverse_tracking)
      points_handler_->trackCandidatesReverse(take_gt_points);

      // track existing candidates
    points_handler_->trackCandidates(take_gt_points);
    if(debug_mapping && frame_current_idx_>=debug_start_frame){
      points_handler_->projectCandidatesOnLastFrame();
      points_handler_->projectActivePointsOnLastFrame();
      points_handler_->showProjectedCandidates( "cands proj");
      points_handler_->showProjectedActivePoints(" act pts proj");

    }
    // activate points
    candidates_activator_->activateCandidates();


  }



  if (cameras_container_->keyframes_active_.size()>2){
    if(remove_occlusions_gt)
      points_handler_->removeOcclusionsInLastKFGrountruth();

    // marginalize points not in last camera
    bundle_adj_->marginalize();
    // bundle adjustment optimization
    bundle_adj_->optimize();
    // // // marginalize points not in last camera
    // // bundle_adj_->marginalize();
  }


  if(use_spectator){
    points_handler_->projectActivePointsOnLastFrame();
    points_handler_->showProjectedActivePoints("last kf ",1);
    spectator_->renderState();
    spectator_->showSpectator(1);
    frame_current_->points_container_->clearProjections();

    if(save_video){
      points_handler_->projectActivePointsOnLastFrame();
      cv::Mat img = spectator_->spectator_image_->image_*255;
      cv::Mat img_out;
      img.convertTo(img_out, CV_8UC3);
      video_spec_.write(img_out);
      Image<colorRGB>* frame = frame_current_->points_container_->getProjectedActivePoints("video");
      frame->image_*=255;
      frame->image_.convertTo(img_out, CV_8UC3);
      video_proj_.write(img_out);
      delete frame;
    }
  }

  if(!kf_added){
    frame_current_->cam_free_mem();
    delete frame_current_->points_container_;
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
