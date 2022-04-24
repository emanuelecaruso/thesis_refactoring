#include "Tracker.h"
#include "CameraForMapping.h"
#include "dso.h"
#include "CamCouple.h"
#include "LinSystem.h"


void Tracker::trackCam(bool groundtruth){
  if(groundtruth){
    dso_->frame_current_->setGrountruthPose();
    sharedCoutDebug("   - Frame tracked (groundtruth)");
  }
  else{
    trackCam();
    sharedCoutDebug("   - Frame tracked");
  }
}

void Tracker::setInitialGuess(){
  if(dso_->parameters_->guess_type==POSE_CONSTANT){
    dso_->frame_current_->assignPose( *(dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_) );
  }
  else if(dso_->parameters_->guess_type==VELOCITY_CONSTANT){

  }
  else{
    throw std::invalid_argument( "initial guess type has wrong value" );
  }
}

void Tracker::trackCam(){

  setInitialGuess();

  std::shared_ptr<CamCoupleContainer> cam_couple_container(new CamCoupleContainer(dso_,ALL_KFS_ON_LAST));
  std::shared_ptr<LinSysTracking> lin_sys_tracking( new LinSysTracking(dso_));

  // iterate through levels
  for (int level=dso_->parameters_->coarsest_level-1; level>=0 ; level--){

    while(true){

      // iterate through keyframes (except last)
      for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
        std::shared_ptr<CameraForMapping> cam_r = dso_->cameras_container_->keyframes_active_[i];

        // select active points vector
        std::vector<std::shared_ptr<ActivePoint>>& active_points = cam_r->points_container_->getActivePoints(level);

        // iterate through active points (at current coarse level)
        for (int j=0; j<active_points.size(); j++){
          std::shared_ptr<ActivePoint> active_point = active_points[j];

          // get measurement
          std::shared_ptr<MeasTracking> measurement(new MeasTracking(active_point, cam_couple_container->get(i,0) ) );
          if(!(measurement->valid)){
            continue;
          }

          // update linear system with that measurement
          lin_sys_tracking->addMeasurement(measurement);
        }
      }

      lin_sys_tracking->updateCameraPose();
      cam_couple_container->update();
      if(dso_->parameters_->debug_tracking){

        std::cout << "chi " << lin_sys_tracking->chi << std::endl;
        dso_->spectator_->renderState();
        dso_->spectator_->showSpectator();
      }
      lin_sys_tracking->clear();

    }

  }


}
