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
      doDso();
    }

  }
}

void Dso::loadFrameCurrent(){

  // sharedCoutDebug("\nFrame "+std::to_string(frame_current_idx_)+" ("+frame_current_->name_+") , frame delay: "+std::to_string(frame_delay));

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
}

void Dso::setFirstKeyframe(){

  tracker_->trackCam(true); // fix first frame to groundtruth pose
  keyframe_handler_->addKeyframe(true);  // add fixed keyframe
  initializer_->extractCorners(); // extract corners from image
  if(debug_initialization){
    initializer_->showCornersTrackCurr();
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
    }
  }
  keyframe_handler_->addKeyframe(true);  // add fixed keyframe
  points_handler_->trackCandidates(take_gt_points); // track existing candidates

  // project candidates and active points on last frame
  candidates_activator_->activateCandidates();
  points_handler_->sampleCandidates(); // sample candidates as high gradient points

  bundle_adj_->optimize(true);

  if(debug_mapping){

    // cameras_container_->keyframes_active_[0]->points_container_->showCandidates();
    // points_handler_->sampleCandidates();
    // points_handler_->showCandidates();
    // points_handler_->projectActivePointsOnLastFrame();

    points_handler_->showProjectedCandidates();
    points_handler_->showProjectedActivePoints();
  }
  to_initialize_=false;
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

  // marginalize points not in last camera
  bundle_adj_->marginalize();
  // bundle adjustment optimization
  bundle_adj_->optimize();


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
  sharedCout("\nVideo stream ended");

}


// bool Dso::makeJsonForActivePts(const std::string& path_name, CameraForMapping* camera){
//
//   // std::cout << "creating json (active pts)" << std::endl;
//   //
//   //   const char* path_name_ = path_name.c_str(); // dataset name
//   //   struct stat info;
//   //   if( stat( path_name_, &info ) != 0 )
//   //   { }
//   //   else if( info.st_mode & S_IFDIR )
//   //   {
//   //     // isdir
//   //     return 0;
//   //   }
//   //   else
//   //   {
//   //     // printf( "%s is not a directory\n", path_name );
//   //     std::string st = "rm " + path_name;
//   //     const char *str = st.c_str();
//   //
//   //   }
//   //
//   //   std::string st = "touch "+path_name;
//   //   const char *str = st.c_str();
//   //
//   //   json j;
//   //
//   //   j["cameras"][camera->name_];
//   //   int count=0;
//   //   for(RegionWithProjActivePoints* reg_proj : *(camera->regions_projected_active_points_->region_vec_) ){
//   //
//   //
//   //     for(int i=0; i<reg_proj->active_pts_proj_vec_->size(); i++){
//   //
//   //       ActivePointProjected* active_pt_proj = reg_proj->active_pts_proj_vec_->at(i);
//   //
//   //       int level = active_pt_proj->level_;
//   //       Eigen::Vector2f uv = active_pt_proj->uv_ ;
//   //       Eigen::Vector3f p;
//   //       camera->pointAtDepth( uv, 1.0/active_pt_proj->invdepth_, p);
//   //       std::stringstream ss;
//   //       ss << std::setw(6) << std::setfill('0') << count;
//   //       std::string idx = ss.str();
//   //       j["cameras"][camera->name_]["p"+idx] = {
//   //         {"level", level},
//   //         {"invdepth_var", active_pt_proj->active_point_->invdepth_var_},
//   //         {"position", {p[0],p[1],p[2]}}
//   //       };
//   //       count++;
//   //
//   //     }
//   //
//   //   }
//   //   // write prettified JSON to another file
//   //   std::ofstream o(path_name);
//   //   o << std::setw(4) << j << std::endl;
//   //   o.close();
//
//     return 1;
// }
//
//
// bool Dso::makeJsonForCameras(const std::string& path_name){
//
//   std::cout << "creating json (cameras)" << std::endl;
//
//     const char* path_name_ = path_name.c_str(); // dataset name
//     struct stat info;
//     if( stat( path_name_, &info ) != 0 )
//     { }
//     else if( info.st_mode & S_IFDIR )
//     {
//       // isdir
//       return 0;
//     }
//     else
//     {
//       // printf( "%s is not a directory\n", path_name );
//       std::string st = "rm " + path_name;
//       const char *str = st.c_str();
//
//     }
//
//     std::string st = "touch "+path_name;
//     const char *str = st.c_str();
//
//     json j;
//
//     const CamParameters* cam_params = camera_vector_->at(0)->cam_parameters_;
//     j["cam_parameters"] = {
//       {"width", cam_params->width*1000},  // width in millimeters
//       {"lens", cam_params->lens*1000},  // lens in millimeters
//       {"min_depth", cam_params->min_depth},
//       {"max_depth", cam_params->max_depth}
//     };
//
//     for(CameraForMapping* camera : *camera_vector_){
//
//       // Eigen::Matrix3f R=camera->frame_camera_wrt_world_->linear();
//       // Eigen::Vector3f t=camera->frame_camera_wrt_world_->translation();
//       Eigen::Isometry3f T = *(camera->frame_camera_wrt_world_);
//
//       // if(camera->name_=="Camera0000"){
//       //   std::cout << T.linear() << std::endl;
//       //   std::cout << T.translation() << std::endl<< std::endl;
//       // }
//       Eigen::Matrix3f R;
//
//       Eigen::Matrix3f flipper;
//       flipper << 1,0,0, 0,-1,0, 0,0,-1;
//       // T.linear()=(flipper*(T.linear().transpose())).transpose();
//       // R=(flipper*(T.linear().transpose())).transpose();
//       R=(flipper*(T.linear().transpose())).transpose();
//
//       // if(camera->name_=="Camera0000"){
//       //   std::cout << T.linear() << std::endl;
//       //   std::cout << T.translation() << std::endl<< std::endl;
//       // }
//
//
//       // T.linear()=flipper*T.linear();
//       Vector6f v;
//       v.head<3>()=T.translation();
//       v[3]=-std::atan2( -R(2,1) , R(2,2) );
//       v[4]=-std::atan2( R(2,0) , sqrt(1-R(2,0)) );
//       v[5]=-std::atan2( -R(1,0) , R(0,0) );
//
//       // if(camera->name_=="Camera0000"){
//       //   std::cout << R << std::endl;
//       //   std::cout << v.head<3>() << std::endl;
//       //   std::cout << v.tail<3>() << std::endl<< std::endl;
//       // }
//
//       j["cameras"][camera->name_];
//
//       // std::stringstream ss;
//       // ss << std::setw(6) << std::setfill('0') << count;
//       // std::string idx = ss.str();
//       j["cameras"][camera->name_] = {
//         {"pose", {v[0],v[1],v[2],v[3],v[4],v[5]}}
//       };
//     }
//
//     // write prettified JSON to another file
//     std::ofstream o(path_name);
//     o << std::setw(4) << j << std::endl;
//     o.close();
//
//     return 1;
//
//
// }
