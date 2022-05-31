#include "CamerasContainer.h"
#include "CameraForMapping.h"
#include "dso.h"


// frame management functions
void CamerasContainer::addFrame(Camera* frame){
  // create CameraForMapping
  CameraForMapping* frame_ = new CameraForMapping(frame);

  frames_.push_back(frame_);
}
void CamerasContainer::addActiveKeyframe(CameraForMapping* keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  keyframes_active_.push_back(keyframe);
  keyframe->keyframe_=true;

}
void CamerasContainer::moveKeyframeToMarginalize(CameraForMapping* keyframe){
  assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  // remove keyframe from active keyframes
  removeActiveKeyframe(keyframe);

  // push keyframe to marginalize
  keyframes_to_marginalize_.push_back(keyframe);


}
void CamerasContainer::moveKeyframeMarginalized(CameraForMapping* keyframe){
  // assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) );
  assert(checkKeyframeToBeMarginalized(keyframe) );
  assert(!checkMarginalizedKeyframe(keyframe));

  // remove keyframe from keyframes to be marginalized
  removeKeyframeToMarginalize(keyframe);

  // push keyframe marginalized
  keyframes_marginalized_.push_back(keyframe);
}
void CamerasContainer::removeFrame(Camera* frame){

  // frames_.push_back(frame);
}
void CamerasContainer::removeActiveKeyframe(CameraForMapping* keyframe){
  // assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) );
  assert(!checkKeyframeToBeMarginalized(keyframe) );
  assert(!checkMarginalizedKeyframe(keyframe));

  removeFromVecByElement(keyframes_active_, keyframe);

}
void CamerasContainer::removeKeyframeToMarginalize(CameraForMapping* keyframe){
  // assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  removeFromVecByElement(keyframes_to_marginalize_, keyframe);

}
void CamerasContainer::removeKeyframeMarginalized(CameraForMapping* keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && checkMarginalizedKeyframe(keyframe));

  removeFromVecByElement(keyframes_marginalized_, keyframe);
}

void CamerasContainer::addFrameWithActPts(CameraForMapping* keyframe){
  assert( checkFrame(keyframe) && !(keyframe->has_active_pts_) );
  keyframe->has_active_pts_=true;
  // push frame
  frames_with_active_pts_.push_back(keyframe);
}

void CamerasContainer::removeFrameWithActPts(CameraForMapping* keyframe){
  assert( checkFrame(keyframe) && checkFrameWithActPts(keyframe) && keyframe->has_active_pts_ );
  keyframe->has_active_pts_=false;
  // remove frame
  removeFromVecByElement(frames_with_active_pts_, keyframe);

}


// check functions
bool CamerasContainer::checkFrame( CameraForMapping* frame ){
  return std::count(frames_.begin(), frames_.end(), frame);
}
bool CamerasContainer::checkFrameWithActPts(CameraForMapping* frame){
  return std::count(frames_with_active_pts_.begin(), frames_with_active_pts_.end(), frame);
}
bool CamerasContainer::checkActiveKeyframe( CameraForMapping* frame ){
  return std::count(keyframes_active_.begin(), keyframes_active_.end(), frame);
}
bool CamerasContainer::checkKeyframeToBeMarginalized( CameraForMapping* frame ){
  return std::count(keyframes_to_marginalize_.begin(), keyframes_to_marginalize_.end(), frame);
}
bool CamerasContainer::checkMarginalizedKeyframe( CameraForMapping* frame ){
  return std::count(keyframes_marginalized_.begin(), keyframes_marginalized_.end(), frame);
}

CameraForMapping* CamerasContainer::getLastActiveKeyframe(){
  return keyframes_active_.back();
}

CameraForMapping* CamerasContainer::getSecondLastFrame(){
  int idx = dso_->frame_current_idx_-1;
  assert(idx>=0 && idx<frames_.size());
  return frames_[idx];
}

CameraForMapping* CamerasContainer::getThirdLastFrame(){
  int idx = dso_->frame_current_idx_-2;
  assert(idx>=0 && idx<frames_.size());
  return frames_[idx];
}
