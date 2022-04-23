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
    // todo
    trackCam();
    sharedCoutDebug("   - Frame tracked");
  }
}

void Tracker::trackCam(){

  std::shared_ptr<CamCoupleContainer> cam_couple_container(new CamCoupleContainer(dso_,ALL_KFS_ON_LAST));
  std::shared_ptr<LinSysTracking> lin_sys_tracking( new LinSysTracking(dso_));

  // iterate through levels
  for (int level=0; level<dso_->parameters_->coarsest_level ; level++){

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

        // update linear system with that measurement
        lin_sys_tracking->addMeasurement(measurement);
      }
    }

  }


}
