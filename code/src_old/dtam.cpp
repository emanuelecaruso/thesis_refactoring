#include "mapper.h"
#include "dtam.h"
#include "json.hpp"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>
#include <fstream>
using json = nlohmann::json;



// void Dso::debugAllCameras(bool show_imgs){
//
//   sharedCoutDebug("DEBUGGING ALL CAMERAS:");
//   sharedCoutDebug("camera vector size: " + std::to_string(camera_vector_->size()));
//
//   for(Camera* camera : *camera_vector_){
//     camera->printMembers();
//     if (show_imgs){
//       camera->showRGB();
//       camera->showDepthMap();
//     }
//   }
//   waitkey(0);
// }
//
// CameraForMapping* Dso::getCurrentCamera() {
//   return camera_vector_->at(frame_current_);
// }
//
// CameraForMapping* Dso::getLastCamera() {
//   return camera_vector_->at(frame_current_-1);
// }
//
// CameraForMapping* Dso::getSecondLastCamera() {
//   return camera_vector_->at(frame_current_-2);
// }
//
// CameraForMapping* Dso::getLastKeyframe() {
//   return camera_vector_->at(keyframe_vector_->back());
// }
//
// PoseNormError* Dso::getTotalPosesNormError(){
//   PoseNormError* poses_norm_error_tot = new PoseNormError;
//   for (int i=0; i<frame_current_+1; i++){
//     CameraForMapping* cam = camera_vector_->at(i);
//     if (cam->keyframe_){
//       PoseNormError cam_pose_norm_error = cam->getPoseNormError();
//       (*poses_norm_error_tot)+=cam_pose_norm_error;
//     }
//   }
//   return poses_norm_error_tot;
// }
//
// float Dso::getTotalPointsNormError(){
//   int count=0;
//   float points_norm_error_tot = 0;
//   for (int i=0; i<frame_current_+1; i++){
//     CameraForMapping* cam = camera_vector_->at(i);
//     if (cam->keyframe_){
//       float cam_point_norm_error = cam->getPointsNormError();
//       points_norm_error_tot+=cam_point_norm_error;
//       count+=cam->active_points_->size();
//     }
//   }
//   return points_norm_error_tot/count;
// }
//
void Dso::addCamera(int counter){

  // CameraForStudy* env_cam=environment_->camera_vector_->at(counter);
  Camera* env_cam=environment_->camera_vector_->at(counter);
  CameraForMapping* new_cam=new CameraForMapping (env_cam, parameters_);
  // new_cam->regions_sampling_->collectCandidates(parameters_->wavelet_levels);
  new_cam->regions_sampling_->collectCandidates(1);
  camera_vector_->push_back(new_cam);
}
//
//
//
void Dso::waitForNewFrame(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  frame_updated_.wait(locker);
  locker.unlock();
}
//
// void Dso::waitForTrackedCandidates(){
//   std::unique_lock<std::mutex> locker(mu_candidate_tracking_);
//   cand_tracked_.wait(locker);
//   locker.unlock();
// }
//
// void Dso::waitForInitialization(){
//   std::unique_lock<std::mutex> locker(mu_initialization_);
//   initialization_done_.wait(locker);
//   locker.unlock();
// }
//
// void Dso::waitForPointActivation(){
//   std::unique_lock<std::mutex> locker(mu_point_activation_);
//   points_activated_.wait(locker);
//   locker.unlock();
// }
//
// void Dso::waitForKeyframeAdded(){
//   std::unique_lock<std::mutex> locker(mu_keyframe_added_);
//   keyframe_added_.wait(locker);
//   locker.unlock();
// }
//
// void Dso::waitForOptimization(){
//   std::unique_lock<std::mutex> locker(mu_optimization_);
//   optimization_done_.wait(locker);
//   locker.unlock();
// }
//
// void Dso::doInitialization(bool initialization_loop, bool debug_initialization, bool debug_mapping, bool track_candidates, bool take_gt_points){
//   bool initialization_done = false;
//
//   while( true ){
//
//     if(frame_current_==int(camera_vector_->size())-1){
//       waitForNewFrame();
//     }
//
//
//     // if still frame current is last camera, new frame is the end signal
//     if(frame_current_==int(camera_vector_->size())-1)
//       break;
//
//     double t_start=getTime();
//
//     frame_current_++;
//
//     int frame_delay = camera_vector_->size()-frame_current_-1;
//     sharedCoutDebug("\nINITIALIZATION of Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));
//
//     if(frame_current_==0){
//       tracker_->trackCam(true);
//       keyframe_handler_->addFixedKeyrame();
//       keyframe_handler_->prepareDataForBA();
//
//       // keyframe_handler_->addKeyframe(true);
//
//       initializer_->extractCorners();
//       if(debug_initialization)
//         initializer_->showCornersTrackCurr();
//       mapper_->selectNewCandidates();
//
//       if(track_candidates)
//         tracker_->collectCandidatesInCoarseRegions();
//
//     }
//     else{
//       initializer_->trackCornersLK();
//
//       // if a good pose is found ...
//       if(initializer_->findPose()){
//         sharedCoutDebug("   - Pose found");
//         if(debug_initialization)
//           initializer_->showCornersTrackCurr();
//
//         if(!initialization_loop){
//           // ... add last keyframe
//           // keyframe_handler_->addKeyframe(true);
//           keyframe_handler_->addFixedKeyrame();
//
//           // start initializing the model
//           // mapper_->trackExistingCandidates(false,debug_mapping);
//           mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
//
//           bundle_adj_->projectActivePoints_prepMarg(0);
//           bundle_adj_->activateNewPoints();
//           bundle_adj_->collectCoarseActivePoints();
//
//           keyframe_handler_->prepareDataForBA();
//
//           keyframe_added_.notify_all();
//
//
//           mapper_->selectNewCandidates();
//
//           if(track_candidates)
//             tracker_->collectCandidatesInCoarseRegions();
//
//           initialization_done=true;
//         }
//
//       }
//     }
//
//     double t_end=getTime();
//     int deltaTime=(t_end-t_start);
//     sharedCoutDebug("   - INITIALIZATION of frame "+std::to_string(frame_current_)+", computation time: "+ std::to_string(deltaTime)+" ms");
//
//     if(initialization_done){
//       sharedCoutDebug("\nINITIALIZATION ENDED");
//       initialization_done_.notify_all();
//       break;
//     }
//
//   }
//
// }
//
// void Dso::doFrontEndPart(bool all_keyframes, bool wait_for_initialization, bool take_gt_poses, bool take_gt_points, bool track_candidates, int guess_type, bool debug_mapping, bool debug_tracking){
//   if(wait_for_initialization)
//     waitForInitialization();
//   while( true ){
//
//     // if the frame is the latest
//     if(frame_current_==int(camera_vector_->size())-1){
//
//       //if cameras are ended
//       if(update_cameras_thread_finished_){
//         break;
//       }
//       else{
//         // std::cout << "FRONT END WAIT NEW FRAME " << std::endl;
//         waitForNewFrame();
//         // std::cout << "FRONT END, NEW FRAME ARRIVED " << std::endl;
//
//       }
//     }
//
//     // // if still frame current is last camera, new frame is the end signal
//     // if(frame_current_==int(camera_vector_->size())-1){
//     //   break;
//     // }
//
//     if (!track_candidates && (wait_for_initialization || frame_current_>=1 ) && bundle_adj_->debug_optimization_ ){
//       std::cout << "FRONT END WAIT FOR OPTIMIZATION " << std::endl;
//       waitForOptimization();
//       std::cout << "FRONT END OPTIMIZATION DONE " << std::endl;
//     }
//
//     double t_start=getTime();
//
//     frame_current_++;
//
//     int frame_delay = camera_vector_->size()-frame_current_-1;
//     sharedCoutDebug("\nFRONT END for Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));
//
//     // if(frame_current_<=1){
//
//       if(frame_current_==0){
//         tracker_->trackCam(true);
//         keyframe_handler_->addFixedKeyrame();
//         mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
//         bundle_adj_->projectActivePoints_prepMarg(0);
//         bundle_adj_->activateNewPoints();
//         bundle_adj_->collectCoarseActivePoints();
//         mapper_->selectNewCandidates();
//         tracker_->collectCandidatesInCoarseRegions();
//         continue;
//       }
//     //   else if(frame_current_==1){
//     //     // mapper_->trackExistingCandidates(true,debug_mapping);
//     //     cand_tracked_.notify_all(); // after candidates are tracked notify for ba
//     //   }
//     //
//     //   if (track_candidates)
//     //
//     //   continue;
//     // }
//
//     // if(frame_current_<=1){
//     //
//     //   if(frame_current_==0){
//     //     tracker_->trackCam(true);
//     //     keyframe_handler_->addFixedKeyrame();
//     //   }
//     //   else if(frame_current_==1){
//     //     mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
//     //     // mapper_->trackExistingCandidates(true,debug_mapping);
//     //     cand_tracked_.notify_all(); // after candidates are tracked notify for ba
//     //   }
//     //   mapper_->selectNewCandidates();
//     //
//     //   if (track_candidates)
//     //     tracker_->collectCandidatesInCoarseRegions();
//     //
//     //   continue;
//     // }
//
//     tracker_->trackCam(take_gt_poses,track_candidates,guess_type,debug_tracking);
//
//     if(keyframe_handler_->addKeyframe(all_keyframes)){
//
//       mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
//
//       bundle_adj_->projectActivePoints_prepMarg(0);
//       bundle_adj_->activateNewPoints();
//       bundle_adj_->collectCoarseActivePoints();
//
//       std::cout << "NEW DATA INCOMING FOR BA, STOP OPT!" << bundle_adj_->keyframe_vector_ba_->size() << "\n";
//       if(bundle_adj_->keyframe_vector_ba_->size()>=2){
//         keyframe_added_flag_=true;
//       }
//       std::unique_lock<std::mutex> locker(mu_restart_opt_);
//       keyframe_handler_->prepareDataForBA();
//       locker.unlock();
//
//       if(bundle_adj_->keyframe_vector_ba_->size()>=2){
//         keyframe_added_flag_=false;
//         keyframe_added_.notify_all();
//       }
//       std::cout << "NEW DATA LOADED, RESTART OPT!" << bundle_adj_->keyframe_vector_ba_->size() << "\n";
//
//
//       // cand_tracked_.notify_all(); // after candidates are tracked notify for ba
//       mapper_->selectNewCandidates();
//       if (track_candidates)
//         tracker_->collectCandidatesInCoarseRegions();
//
//     }
//     double t_end=getTime();
//     int deltaTime=(t_end-t_start);
//     sharedCoutDebug("FRONT END part of frame: "+std::to_string(frame_current_)+", time: "+ std::to_string(deltaTime)+" ms");
//
//   }
//
//   frontend_thread_finished_=true;
//   cand_tracked_.notify_all();
//
// }
//
// void Dso::setOptimizationFlags( bool debug_optimization, int opt_norm, int test_single, int image_id, bool test_marginalization){
//   bundle_adj_->debug_optimization_=debug_optimization;
//   bundle_adj_->opt_norm_=opt_norm;
//   bundle_adj_->test_single_=test_single;
//   bundle_adj_->image_id_=image_id;
//   bundle_adj_->test_marginalization_=test_marginalization;
// }
//
// void Dso::noiseToPoses(float range_angle, float range_position){
//
//   for(int i=0; i<bundle_adj_->keyframe_vector_ba_->size() ; i++){
//     CameraForMapping* keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i));
//     if(keyframe->fixed_){
//       std::cout << keyframe->name_ << " FIRST KEYFRAME " << std::endl;
//       continue;
//     }
//
//     Vector6f noise;
//     float t0 = randZeroMeanNoise(range_position); float t1 = randZeroMeanNoise(range_position); float t2 = randZeroMeanNoise(range_position);
//     float t3 = randZeroMeanNoise(range_angle); float t4 = randZeroMeanNoise(range_angle); float t5 = randZeroMeanNoise(range_angle);
//     noise << t0, t1, t2, t3, t4, t5;
//     std::cout << keyframe->name_ << ", NOISE, positions: " << t0 << ", " << t1 << ", " << t2 << " , angles: " << t3 << ", " << t4 << ", " << t5 << std::endl;
//     Eigen::Isometry3f frame_camera_wrt_world_noisy = (*(keyframe->grountruth_camera_->frame_camera_wrt_world_))*v2t_inv(noise);
//     keyframe->assignPose0(frame_camera_wrt_world_noisy);
//     keyframe->assignPose(frame_camera_wrt_world_noisy);
//
//   }
// }
//
// Eigen::VectorXf* Dso::noiseToPosesSame(float range_angle, float range_position){
//
//   Eigen::VectorXf* dx_poses_marg = new Eigen::VectorXf(bundle_adj_->hessian_b_marg->pose_block_size) ;
//
//   // noise
//   Vector6f noise; // x0 wrt x, x_T_x0
//   float t0 = range_position; float t1 = range_position; float t2 = range_position;
//   float t3 = range_angle; float t4 = range_angle; float t5 = range_angle;
//   noise << t0, t1, t2, t3, t4, t5;
//
//
//   // collect relative transformations
//   std::vector<Eigen::Isometry3f> relative_transformations;
//   for(int i=1; i<bundle_adj_->keyframe_vector_ba_->size()-1 ; i++){
//
//     // get relative transformations
//     CameraForMapping* keyframe_prev = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i));
//     CameraForMapping* keyframe_next = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i+1));
//     Eigen::Isometry3f prev_T_next = (*keyframe_prev->frame_world_wrt_camera_)*(*keyframe_next->frame_camera_wrt_world_);
//     relative_transformations.push_back(prev_T_next);
//
//   }
//
//   // noise to first keyframe
//   CameraForMapping* keyframe_first = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(1));
//   assert(!(keyframe_first->fixed_) );
//   assert(!(keyframe_first->to_be_marginalized_ba_) );
//   Eigen::Isometry3f frame_camera_wrt_world_noisy =  (*(keyframe_first->grountruth_camera_->frame_camera_wrt_world_))*v2t_inv(noise);
//   keyframe_first->assignPose0(frame_camera_wrt_world_noisy);
//   keyframe_first->assignPose(frame_camera_wrt_world_noisy);
//   // update delta
//   Vector6f segment = noise;
//   int block_idx_marg = keyframe_first->state_pose_block_marg_idx_;
//   if (block_idx_marg!=-1)
//     dx_poses_marg->segment<6>(block_idx_marg)=segment;
//
//   // match relative transformations
//   for(int i=1; i<bundle_adj_->keyframe_vector_ba_->size()-1 ; i++){
//     CameraForMapping* keyframe_prev = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i));
//     CameraForMapping* keyframe_next = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i+1));
//     Eigen::Isometry3f prev_T_next = relative_transformations[i-1];
//
//     // update delta
//     int block_idx_marg = keyframe_next->state_pose_block_marg_idx_;
//     if (block_idx_marg!=-1){
//       Eigen::Isometry3f w_T_cam0 = *(keyframe_next->frame_camera_wrt_world_); // current w_T_cam0
//       Eigen::Isometry3f w_T_cam = (*(keyframe_prev->frame_camera_wrt_world_))*prev_T_next; // next w_T_cam
//       Vector6f segment = t2v(w_T_cam.inverse()*w_T_cam0); // noise x_T_x0
//       dx_poses_marg->segment<6>(block_idx_marg)=segment;
//     }
//
//     //assign to keyframe next
//     keyframe_next->assignPose0( (*(keyframe_prev->frame_camera_wrt_world_))*prev_T_next );
//     keyframe_next->assignPose( (*(keyframe_prev->frame_camera_wrt_world_))*prev_T_next );
//
//   }
//
//   return dx_poses_marg;
//
// }
//
// void Dso::noiseToPoints(float range_invdepth){
//
//   float invdepth_err = range_invdepth;
//
//   for(int i=0; i<bundle_adj_->keyframe_vector_ba_->size() ; i++){
//     CameraForMapping* keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(i));
//     // iterate through all active points
//     for(int j=0; j<keyframe->active_points_->size() ; j++){
//       ActivePoint* active_pt = keyframe->active_points_->at(j);
//       // float invdepth_err = randZeroMeanNoise(range_invdepth);
//       float invdepth_gt = active_pt->getInvdepthGroundtruth();
//       active_pt->invdepth_0_=invdepth_gt+invdepth_err;
//       active_pt->invdepth_=invdepth_gt+invdepth_err;
//       active_pt->invdepth_var_=(range_invdepth*range_invdepth)+0.001;
//       // since change both poses and invdepths
//       keyframe->pointAtDepthInCamFrame(active_pt->uv_, 1.0/active_pt->invdepth_, *(active_pt->p_incamframe_));
//       *(active_pt->p_)=(*(keyframe->frame_camera_wrt_world_0_))*v2t_inv(*(keyframe->delta_update_x_))*(*(active_pt->p_incamframe_));
//     }
//
//   }
// }
//
// void Dso::doOptimization(bool active_all_candidates, bool debug_optimization, int opt_norm, int test_single, int image_id, bool test_marginalization){
//
//   setOptimizationFlags(debug_optimization,opt_norm,test_single,image_id,test_marginalization);
//
//   while( true ){
//
//     if(!frontend_thread_finished_){
//       std::cout << "OPTIMIZATION, WAIT NEW POINT ACTIVATION  " << std::endl;
//       waitForKeyframeAdded();
//       std::cout << "OPTIMIZATION, STOP WAITING NEW POINT ACTIVATION " << keyframe_added_flag_ << std::endl;
//     }
//     else
//       break;
//
//     if(bundle_adj_->keyframe_vector_ba_->size()<=2){
//       // int test_single = bundle_adj_->test_single_;
//       // bundle_adj_->test_single_=TEST_ONLY_POINTS;
//       // bundle_adj_->optimize();
//       // bundle_adj_->test_single_=test_single;
//       // if(debug_optimization){
//         std::this_thread::sleep_for(std::chrono::microseconds(10000));
//         optimization_done_.notify_all();
//       // }
//
//       continue;
//     }
//
//
//     // DEBUG
//     if(bundle_adj_->debug_optimization_){
//       // // cam on which active points are projected
//       // CameraForMapping* last_keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->back());
//       // last_keyframe->showProjActivePoints(1);
//       // // last_keyframe->showProjCandidates(1);
//       //
//       //
//       // // iterate along all cameras
//       // // for (int j=0; j<int(bundle_adj_->keyframe_vector_ba_->size())-1; j++){
//       // //     CameraForMapping* keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(j));
//       // //     for (int i=keyframe->candidates_coarse_->size(); i>0; i--){
//       // //         // std::cout << "CAMERA " << keyframe->name_ << std::endl;
//       // //         keyframe->showCoarseActivePoints(i,1);}
//       // //     }
//       // //
//       //   waitkey(0);
//       //   cv::destroyAllWindows();
//     }
//
//     if(test_single==TEST_ONLY_POSES || test_single==TEST_ONLY_POSES_ONLY_M){
//       // noise on cam poses
//       // noiseToPoses(1./180., 0.01);
//       noiseToPoses(3./180., 0.02);
//       // noiseToPoses(0, 0);
//
//     }
//     else if(test_single==TEST_ONLY_POINTS){
//       // noiseToPoints(0.25);
//       noiseToPoints(0.05);
//       // noiseToPoints(0.01);
//       // noiseToPoints(0);
//     }
//
//     // optimize
//     bundle_adj_->optimize();
//
//     if(debug_optimization){
//       // after optimization
//       // std::cout << "OPTIMIZATION WAIT NEW FRAME " << std::endl;
//       if(!update_cameras_thread_finished_ )
//         waitForNewFrame();
//       // std::cout << "OPTIMIZATION, NEW FRAME ARRIVED " << std::endl;
//
//
//       // time guard
//       std::this_thread::sleep_for(std::chrono::microseconds(10000));
//
//
//       // notify all threads that optimization has been done
//       optimization_done_.notify_all();
//     }
//
//   }
// }
//
void Dso::setFirstKeyframe(){


  // camera management
  tracker_->trackCam(true); // take groundtruth pose
  keyframe_handler_->addFixedKeyrame(); // first frame is fixed

  // for initialization
  initializer_->extractCorners();
  if(parameters_->debug_initialization)
   initializer_->showCornersTrackCurr();
  mapper_->selectNewCandidates();

  first_frame_to_set_=false;
}

void Dso::updateCurrentFrame(){
  if(frame_current_==camera_vector_->size()-1){
    waitForNewFrame();
  }
  if(parameters_->get_current_frame)
    frame_current_=camera_vector_->size()-1;
  else
    frame_current_++;
}


void Dso::doDSOSequential(){

  while(true){
    updateCurrentFrame();

    if(first_frame_to_set_){
      setFirstKeyframe();
    }
    else if(to_initialize_){
      initialize();
    }
    else{
      // doDso();
    }
  }




  //   // if the frame is the latest
  //   if(frame_current_==int(camera_vector_->size())-1){
  //
  //     //if cameras are ended
  //     if(update_cameras_thread_finished_){
  //       break;
  //     }
  //     else{
  //       waitForNewFrame();
  //     }
  //   }
  //
  //
  //   double t_start=getTime();
  //   frame_current_++;
  //   int frame_delay = camera_vector_->size()-frame_current_-1;
  //
  //   // initialization
  //   if(!initialization_done){
  //     sharedCoutDebug("\nINITIALIZATION of Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));
  //
  //     if(frame_current_==0){
  //
  //       // camera management
  //       tracker_->trackCam(true); // groundtruth
  //       keyframe_handler_->addFixedKeyrame(); // first frame is fixed
  //       keyframe_handler_->prepareDataForBA();
  //
  //       // initialization
  //       initializer_->extractCorners();
  //       if(debug_initialization)
  //         initializer_->showCornersTrackCurr();
  //       mapper_->selectNewCandidates();
  //
  //     }
  //     else{
  //       initializer_->trackCornersLK();
  //
  //       // if a good pose is found ...
  //       if(initializer_->findPose()){
  //         sharedCoutDebug("   - Pose found");
  //         if(debug_initialization)
  //           initializer_->showCornersTrackCurr();
  //
  //         // ... add last keyframe
  //         // keyframe_handler_->addKeyframe(true);
  //         keyframe_handler_->addFixedKeyrame();
  //
  //         // start initializing the model
  //         mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
  //
  //         bundle_adj_->projectActivePoints_prepMarg(0);
  //         bundle_adj_->activateNewPoints();
  //         bundle_adj_->collectCoarseActivePoints();
  //
  //         keyframe_handler_->prepareDataForBA();
  //         int test_single = bundle_adj_->test_single_;
  //         bundle_adj_->test_single_=TEST_ONLY_POINTS;
  //         bundle_adj_->optimize();
  //         bundle_adj_->test_single_=test_single;
  //
  //         mapper_->selectNewCandidates();
  //
  //         initialization_done=true;
  //
  //         double t_end=getTime();
  //         int deltaTime=(t_end-t_start);
  //         sharedCoutDebug("   - INITIALIZATION of frame "+std::to_string(frame_current_)+", computation time: "+ std::to_string(deltaTime)+" ms");
  //         if(initialization_done){
  //           sharedCoutDebug("\nINITIALIZATION ENDED");
  //         }
  //
  //       }
  //     }
  //
  //   }
  //   // front end part
  //   else{
  //
  //       sharedCoutDebug("\nFRONT END for Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));
  //
  //
  //       tracker_->trackCam(take_gt_poses,0,guess_type,debug_tracking);
  //
  //       if(keyframe_handler_->addKeyframe(all_keyframes)){
  //
  //         mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
  //
  //         bundle_adj_->projectActivePoints_prepMarg(0);
  //         bundle_adj_->activateNewPoints();
  //         bundle_adj_->collectCoarseActivePoints();
  //
  //         keyframe_handler_->prepareDataForBA();
  //         bundle_adj_->optimize();
  //
  //         mapper_->selectNewCandidates();
  //       }
  //       double t_end=getTime();
  //       int deltaTime=(t_end-t_start);
  //       sharedCoutDebug("FRONT END part of frame: "+std::to_string(frame_current_)+", time: "+ std::to_string(deltaTime)+" ms");
  //
  //   }
  //
  //
  //
  // }
}

// void Dso::doSpect(){
//   spectator_->spectateDso();
// }

void Dso::updateCamerasFromEnvironment(){

  assert(environment_->camera_vector_->size()!=0);

  float fps=environment_->fps_;
  int counter=0;

  while(counter< environment_->camera_vector_->size()){

    // std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    // sharedCout("\nFrame: "+ std::to_string(counter));
    addCamera(counter);

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
  update_cameras_thread_finished_=true;
  frame_updated_.notify_all();
  sharedCout("\nVideo stream ended");

}
//
//
// void Dso::eval_initializer(){
//
//   // bool debug_initialization=true;
//   // bool debug_mapping=false;
//   // bool track_candidates=false;
//   //
//   // bool initialization_loop=true;
//   //
//   // std::thread initialization_thread_(&Dso::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates );
//   // std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
//   //
//   // update_cameras_thread_.join();
//   // initialization_thread_.join();
//   //
//   // // initializer_->showCornersRef();
//   // initializer_->showCornersTrackSequence();
//   // waitkey(0);
//
// }
//
// // void Dso::test_mapping(){
// //
// //   bool debug_mapping=true;
// //   bool debug_tracking=false;
// //
// //   bool take_gt_poses=true;
// //   bool take_gt_points=false;
// //   bool track_candidates=true;
// //
// //   int guess_type=VELOCITY_CONSTANT;
// //
// //   bool all_keyframes=true;
// //   bool wait_for_initialization=false;
// //
// //   std::thread frontend_thread_(&Dso::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
// //   std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
// //
// //
// //   // optimization_thread.join();
// //   frontend_thread_.join();
// //   update_cameras_thread_.join();
// //
// //
// //   waitkey(0);
// //
// // }
// //
// //
// // void Dso::test_tracking(){
// //
// //   bool debug_mapping=false;
// //   bool debug_tracking=true;
// //
// //   // tracking flags
// //   bool take_gt_poses=false;
// //   // bool take_gt_points=true;
// //   bool take_gt_points=false;
// //   bool track_candidates=true;
// //   int guess_type=POSE_CONSTANT;
// //
// //   bool all_keyframes=true;
// //   bool wait_for_initialization=false;
// //
// //   std::thread frontend_thread_(&Dso::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
// //   std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
// //
// //   // optimization_thread.join();
// //   frontend_thread_.join();
// //   update_cameras_thread_.join();
// //
// //
// //   waitkey(0);
// //
// // }
// //
// //
// // void Dso::test_optimization_pose(){
// //
// //   bool debug_initialization=false;
// //   bool debug_mapping=false;
// //   bool debug_tracking=false;
// //   bool debug_optimization= true;
// //
// //   bool initialization_loop=false;
// //   bool take_gt_poses=false;
// //   bool take_gt_points=true;
// //
// //   bool track_candidates=false;
// //   // int guess_type=POSE_CONSTANT;
// //   int guess_type=VELOCITY_CONSTANT;
// //   // int opt_norm=HUBER;
// //   int opt_norm=QUADRATIC;
// //   int test_single=TEST_ONLY_POSES;
// //   // int image_id=INTENSITY_ID;
// //   int image_id=GRADIENT_ID;
// //   bool test_marginalization=false;
// //
// //   bool all_keyframes=true;
// //   bool wait_for_initialization=false;
// //   bool active_all_candidates=true;
// //
// //   std::thread frontend_thread_(&Dso::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
// //   // std::thread initialization_thread_(&Dso::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates, take_gt_points);
// //   std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
// //
// //   if(!track_candidates){
// //     std::thread optimization_thread(&Dso::doOptimization, this, active_all_candidates, debug_optimization,opt_norm, test_single, image_id, test_marginalization);
// //     optimization_thread.join();
// //   }
// //
// //   // initialization_thread_.join();
// //   update_cameras_thread_.join();
// //   frontend_thread_.join();
// //
// //   // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));
// //   // makeJsonForActivePts("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(3));
// //   makeJsonForCameras("./dataset/"+environment_->dataset_name_+"/state_cameras.json");
// //
// //
// // }
// //
// //
// // void Dso::test_optimization_points(){
// //
// //   bool debug_initialization=false;
// //   bool debug_mapping=false;
// //   bool debug_tracking=false;
// //   bool debug_optimization= true;
// //
// //   bool initialization_loop=false;
// //   bool take_gt_poses=true;
// //   bool take_gt_points=true;
// //
// //   bool track_candidates=false;
// //   // int guess_type=POSE_CONSTANT;
// //   int guess_type=VELOCITY_CONSTANT;
// //   // int opt_norm=HUBER;
// //   int opt_norm=QUADRATIC;
// //   int test_single=TEST_ONLY_POINTS;
// //   // int image_id=INTENSITY_ID;
// //   int image_id=GRADIENT_ID;
// //   bool test_marginalization=false;
// //
// //   bool all_keyframes=true;
// //   bool wait_for_initialization=true;
// //   bool active_all_candidates=true;
// //
// //   std::thread frontend_thread_(&Dso::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
// //   std::thread initialization_thread_(&Dso::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates, take_gt_points);
// //   std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
// //
// //   if(!track_candidates){
// //     std::thread optimization_thread(&Dso::doOptimization, this, active_all_candidates, debug_optimization,opt_norm, test_single, image_id, test_marginalization);
// //     optimization_thread.join();
// //   }
// //
// //   initialization_thread_.join();
// //   update_cameras_thread_.join();
// //   frontend_thread_.join();
// //
// //   // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));
// //   makeJsonForActivePts("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));
// //
// //
// // }
// //
// // void Dso::test_dso(){
// //
// //   bool debug_initialization=false;
// //   bool debug_mapping=false;
// //   bool debug_tracking=true;
// //   bool debug_optimization= true;
// //
// //   bool initialization_loop=false;
// //   bool take_gt_poses=false;
// //   bool take_gt_points=false;
// //
// //   bool track_candidates=false;
// //   int guess_type=POSE_CONSTANT;
// //   // int guess_type=VELOCITY_CONSTANT;
// //   int opt_norm=HUBER;
// //   // int opt_norm=QUADRATIC;
// //   int test_single=TEST_ALL;
// //   // int image_id=INTENSITY_ID;
// //   int image_id=GRADIENT_ID;
// //   // int image_id=PHASE_ID;
// //   bool test_marginalization=false;
// //
// //   bool all_keyframes=false;
// //   bool wait_for_initialization=true;
// //   bool active_all_candidates=true;
// //
// //   bool show_spectator=true;
// //
// //   std::thread frontend_thread_(&Dso::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
// //   std::thread initialization_thread_(&Dso::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates, take_gt_points);
// //   std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
// //   std::thread optimization_thread(&Dso::doOptimization, this, active_all_candidates, debug_optimization,opt_norm, test_single, image_id, test_marginalization);
// //   // std::thread spectator_thread(&Dso::doSpect, this);
// //
// //   // spectator_thread.join();
// //   optimization_thread.join();
// //   initialization_thread_.join();
// //   update_cameras_thread_.join();
// //   frontend_thread_.join();
// //
// //   makeJsonForCameras("./dataset/"+environment_->dataset_name_+"/state_cameras.json");
// //   makeJsonForActivePts("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));
// //
// //
// // }
//
//
void Dso::test_dso_sequential(){

  bool debug_initialization=parameters_->debug_initialization;
  bool debug_mapping=parameters_->debug_mapping;
  bool debug_tracking=parameters_->debug_tracking;
  bool debug_optimization=parameters_->debug_optimization;
  bool take_gt_poses=parameters_->take_gt_poses;
  bool take_gt_points=parameters_->take_gt_points;
  int guess_type=parameters_->guess_type;
  int opt_norm=parameters_->opt_norm;
  int test_single=parameters_->test_single;
  int image_id=parameters_->image_id;
  bool test_marginalization=parameters_->test_marginalization;
  bool all_keyframes=parameters_->all_keyframes;
  bool active_all_candidates=parameters_->active_all_candidates;
  bool show_spectator=parameters_->show_spectator;

  std::thread update_cameras_thread_(&Dso::updateCamerasFromEnvironment, this);
  std::thread dso_thread_(&Dso::doDSOSequential, this );
  // std::thread dso_thread_(&Dso::doDSOSequential, this);
  // std::thread spectator_thread(&Dso::doSpect, this);

  // spectator_thread.join();
  dso_thread_.join();
  update_cameras_thread_.join();

  // makeJsonForCameras("./dataset/"+environment_->dataset_name_+"/state_cameras.json");
  // makeJsonForActivePts("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));


}
//
// // TODO remove
// bool Dso::makeJsonForCands(const std::string& path_name, CameraForMapping* camera){
//
//   std::cout << "creating json (cands)" << std::endl;
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
//     j["cameras"][camera->name_];
//     int count=0;
//     for(RegionWithProjCandidates* reg_proj : *(camera->regions_projected_cands_->region_vec_) ){
//
//
//       for(int i=0; i<reg_proj->cands_proj_vec_->size(); i++){
//
//         CandidateProjected* cand_proj = reg_proj->cands_proj_vec_->at(i);
//
//         int level = cand_proj->level_;
//         Eigen::Vector2f uv = cand_proj->uv_ ;
//         Eigen::Vector3f p;
//         camera->pointAtDepth( uv, 1.0/cand_proj->invdepth_, p);
//         std::stringstream ss;
//         ss << std::setw(6) << std::setfill('0') << count;
//         std::string idx = ss.str();
//         j["cameras"][camera->name_]["p"+idx] = {
//           {"level", level},
//           {"invdepth_var", cand_proj->cand_->invdepth_var_},
//           {"position", {p[0],p[1],p[2]}}
//         };
//         count++;
//
//       }
//
//     }
//     // write prettified JSON to another file
//     std::ofstream o(path_name);
//     o << std::setw(4) << j << std::endl;
//     o.close();
//
//     return 1;
//
//
// }
//
//
// // TODO remove
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
