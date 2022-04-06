#include "CamerasContainer.h"

// frame management functions
void CamerasContainer::addFrame(std::shared_ptr<Camera> frame){

  // frames_.push_back(frame);
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
