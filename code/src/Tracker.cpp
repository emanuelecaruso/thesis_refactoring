#include "Tracker.h"
#include "CameraForMapping.h"
#include "dso.h"

void Tracker::trackCam(bool groundtruth){
  if(groundtruth){
    dso_->frame_current_->setGrountruthPose();
    sharedCoutDebug("   - Frame tracked (groundtruth)");
  }
  else{
    // todo
  }
}
