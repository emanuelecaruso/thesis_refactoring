#include "Tree.h"
#include "CameraForMapping.h"

void Tree::checkTreeValidity(){
  std::shared_ptr<Image<pixelIntensity>> image = cam_->image_intensity_;
  int subd = pow(2,levels_);
  assert( !(image->image_.rows%subd ) || !(image->image_.cols%subd ) );
}

void TreeCandSel::init(){
  checkTreeValidity();
  // iterate along all pixels
}

void TreeActPtAct::init(){
  checkTreeValidity();
  // iterate along all pixels 4 by 4,
}

void TreeActPtCoarseGen::init(){
  checkTreeValidity();
  // iterate along all pixels 4 by 4,
}
