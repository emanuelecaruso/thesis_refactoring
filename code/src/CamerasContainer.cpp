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

  int v_size = keyframes_active_.size();
  keyframes_active_.erase(std::remove(keyframes_active_.begin(), keyframes_active_.end(), keyframe), keyframes_active_.end());
  assert(v_size==keyframes_active_.size()+1);

}
void CamerasContainer::removeKeyframeToMarginalize(CameraForMapping* keyframe){
  // assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  int v_size = keyframes_to_marginalize_.size();
  keyframes_to_marginalize_.erase(std::remove(keyframes_to_marginalize_.begin(), keyframes_to_marginalize_.end(), keyframe), keyframes_to_marginalize_.end());
  assert(v_size==keyframes_to_marginalize_.size()+1);

}
void CamerasContainer::removeKeyframeMarginalized(CameraForMapping* keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && checkMarginalizedKeyframe(keyframe));

  int v_size = keyframes_marginalized_.size();
  keyframes_marginalized_.erase(std::remove(keyframes_marginalized_.begin(), keyframes_marginalized_.end(), keyframe), keyframes_marginalized_.end());
  assert(v_size==keyframes_marginalized_.size()+1);
}

// check functions
bool CamerasContainer::checkFrame( CameraForMapping* frame ){
  return std::count(frames_.begin(), frames_.end(), frame);
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
