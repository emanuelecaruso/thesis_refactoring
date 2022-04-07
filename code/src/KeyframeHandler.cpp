#include "KeyframeHandler.h"
#include "CameraForMapping.h"
#include "dso.h"

void KeyframeHandler::addKeyframe(bool fixed){

  dso_->cameras_container_.addActiveKeyframe(dso_->frame_current_);
  dso_->frame_current_->fixed_=fixed;
  sharedCoutDebug("   - First keyframe added");

}
