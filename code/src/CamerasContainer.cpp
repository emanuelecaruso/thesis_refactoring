#include "CamerasContainer.h"
#include "CameraForMapping.h"
#include "dso.h"

// frame management functions
void CamerasContainer::addFrame(std::shared_ptr<Camera> frame){
  // create CameraForMapping
  std::shared_ptr<CameraForMapping> frame_ ( new CameraForMapping(frame,dso_->parameters_));

  frames_.push_back(frame_);
}
void CamerasContainer::addActiveKeyframe(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  keyframes_active_.push_back(keyframe);

}
void CamerasContainer::addKeyframeToMarginalize(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  keyframes_to_marginalize_.push_back(keyframe);

}
void CamerasContainer::addKeyframeMarginalized(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));
  keyframes_marginalized_.push_back(keyframe);
}
void CamerasContainer::removeFrame(std::shared_ptr<Camera> frame){

  // frames_.push_back(frame);
}
void CamerasContainer::removeActiveKeyframe(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  keyframes_active_.erase(std::remove(keyframes_active_.begin(), keyframes_active_.end(), keyframe), keyframes_active_.end());
}
void CamerasContainer::removeKeyframeToMarginalize(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && checkActiveKeyframe(keyframe) && !checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  keyframes_active_.erase(std::remove(keyframes_to_marginalize_.begin(), keyframes_to_marginalize_.end(), keyframe), keyframes_to_marginalize_.end());
}
void CamerasContainer::removeKeyframeMarginalized(std::shared_ptr<CameraForMapping> keyframe){
  assert(checkFrame(keyframe) && !checkActiveKeyframe(keyframe) && checkKeyframeToBeMarginalized(keyframe) && !checkMarginalizedKeyframe(keyframe));

  keyframes_marginalized_.erase(std::remove(keyframes_marginalized_.begin(), keyframes_marginalized_.end(), keyframe), keyframes_marginalized_.end());
}

// check functions
bool CamerasContainer::checkFrame( std::shared_ptr<CameraForMapping> frame ){
  return std::count(frames_.begin(), frames_.end(), frame);
}
bool CamerasContainer::checkActiveKeyframe( std::shared_ptr<CameraForMapping> frame ){
  return std::count(keyframes_active_.begin(), keyframes_active_.end(), frame);
}
bool CamerasContainer::checkKeyframeToBeMarginalized( std::shared_ptr<CameraForMapping> frame ){
  return std::count(keyframes_to_marginalize_.begin(), keyframes_to_marginalize_.end(), frame);
}
bool CamerasContainer::checkMarginalizedKeyframe( std::shared_ptr<CameraForMapping> frame ){
  return std::count(keyframes_marginalized_.begin(), keyframes_marginalized_.end(), frame);
}

std::shared_ptr<CameraForMapping> CamerasContainer::getLastActiveKeyframe(){
  return keyframes_active_.back();
}
