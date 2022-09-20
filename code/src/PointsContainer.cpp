#include "PointsContainer.h"
#include "CamCouple.h"
#include "BundleAdj.h"
#include <algorithm>    // std::max

float Point::getInvdepthErr(){
  return abs(invdepth_-getInvdepthGroundtruth())/invdepth_var_;
}

void Candidate::showCandidate(){
  cam_->pyramid_->getC(level_)->showImgWithColoredPixel( pixel_, pow(2,level_)+1, cam_->name_+", cand");
}

void Candidate::setInvdepthGroundtruth(){

  invdepth_=getInvdepthGroundtruth();
  invdepth_var_=0.0001;
}

float Point::getInvdepthGroundtruth(){
  pxl pixel;
  cam_->uv2pixelCoords(uv_, pixel);
  float invdepth_val = cam_->grountruth_camera_->invdepth_map_->evalPixel(pixel);
  float invdepth_gt = invdepth_val/cam_->cam_parameters_->min_depth;
  return invdepth_gt;
}

void Candidate::remove(){
  // remove candidate from vector
  removeFromVecByElement(cam_->points_container_->candidates_, this);

  delete this;
}



void PointsContainer::drawPoint(Point* point, Image<colorRGB>* show_img, bool circle){
  pxl pixel= point->pixel_;
  colorRGB color = black;
  if( point->invdepth_>0 )
    color = cam_->cam_parameters_->invdepthToRgb(point->invdepth_);
  if(circle)
    show_img->drawCircle( color, pixel, 1, 2);
  else
    show_img->setPixel( pixel,color);
}

void PointsContainer::showCandidates(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(candidates_.size())+" candidates";
  Image<colorRGB>* show_img( cam_->pyramid_->getC(candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<candidates_.size(); i++){
    Candidate* candidate = candidates_.at(i);
    drawPoint(candidate,show_img);
  }

  show_img->show(pow(2,candidate_level));
  cv::waitKey(0);

}

void PointsContainer::showProjectedCandidates( ){

  std::string name = cam_->name_+" , "+std::to_string(candidates_projected_.size())+" projected candidates";
  showProjectedCandidates( name );

}

void PointsContainer::showProjectedCandidates( const std::string& name ){
  double alpha = 1;

  Image<colorRGB>* show_img( cam_->pyramid_->getC(candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<candidates_projected_.size(); i++){
    CandidateProjected* candidate_proj = candidates_projected_.at(i);
    drawPoint(candidate_proj,show_img);
  }

  show_img->show(pow(2,candidate_level));
  cv::waitKey(1);

}



void PointsContainer::showActivePoints(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(active_points_.size())+" active points";
  Image<colorRGB>* show_img( cam_->pyramid_->getC(candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<active_points_.size(); i++){
    ActivePoint* active_pt = active_points_.at(i);
    drawPoint(active_pt,show_img);
  }

  show_img->show(pow(2,candidate_level));
  cv::waitKey(0);

}

Image<colorRGB>* PointsContainer::getProjectedActivePoints(const std::string& name){
  Image<colorRGB>* show_img( cam_->pyramid_->getC(candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through active points projected
  for(int i=0; i<active_points_projected_.size(); i++){
    ActivePointProjected* active_pt_proj = active_points_projected_[i];
    drawPoint(active_pt_proj,show_img);
  }

  return show_img;
}


void PointsContainer::showProjectedActivePoints( int wtk){
  std::string name = cam_->name_+" , "+std::to_string(active_points_projected_.size())+" projected active points";
  showProjectedActivePoints(name,wtk);
}

void PointsContainer::showProjectedActivePoints(const std::string& name, int wtk){

  Image<colorRGB>* show_img = getProjectedActivePoints(name);

  show_img->show(pow(1,candidate_level));
  cv::waitKey(wtk);
  delete show_img;

}

void PointsContainer::clearProjections(){
  for( CandidateProjected* cand_proj : candidates_projected_ )
    delete cand_proj;
  for( ActivePointProjected* active_pt_proj : active_points_projected_ )
    delete active_pt_proj;
  for( MarginalizedPointProjected* marg_pt_proj : marginalized_points_projected_ )
    delete marg_pt_proj;
  candidates_projected_.clear();
  active_points_projected_.clear();
  marginalized_points_projected_.clear();
}



// void CandidateProjected::init(Candidate* cand, std::shared_ptr<CamCouple> cam_couple_){
//   float d2;
//   cam_couple_->reprojection(cand->uv_,1./cand->invdepth_,uv_,d2);
//   cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, cand->level_);
//   cam_=cam_couple_->cam_m_;
//   level_=cand->level_;
//   invdepth_=1.0/d2;
// }
//
//
// void ActivePointProjected::init(ActivePoint* active_pt, std::shared_ptr<CamCouple> cam_couple_){
//   float d2;
//   cam_couple_->reprojection(active_pt->uv_,1./active_pt->invdepth_,uv_,d2);
//   cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, active_pt->level_);
//   cam_=cam_couple_->cam_m_;
//   level_=active_pt->level_;
//   invdepth_=1.0/d2;
// }

void CandidateProjected::init(Candidate* cand, std::shared_ptr<CamCouple> cam_couple_){

  assert(cand->invdepth_!=-1);
  // self projections
  if(cam_couple_->cam_r_==cam_couple_->cam_m_){
    uv_.x()=cand->uv_.x();
    uv_.y()=cand->uv_.y();
    invdepth_=cand->invdepth_;
    level_=cand->level_;
    cam_=cam_couple_->cam_m_;
  }

  level_=cand->level_;
  cam_=cam_couple_->cam_m_;

  // cam_couple_->getUv( cand->uv_.x(),cand->uv_.y(),1./cand->invdepth_,uv_.x(),uv_.y() );
  // cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, cand->level_);
  // float depth;
  // cam_couple_->getD2(cand->uv_.x(),cand->uv_.y(),1./cand->invdepth_,depth);
  // invdepth_=1./depth;

  float depth;
  cam_couple_->reprojection(cand->uv_, (1./cand->invdepth_) , uv_, depth);
  invdepth_=1./depth;
  cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, cand->level_);
}


void ActivePointProjected::init(ActivePoint* active_pt, std::shared_ptr<CamCouple> cam_couple_){
  float depth;
  cam_couple_->reprojection(active_pt->uv_, (1./active_pt->invdepth_) , uv_, depth);
  invdepth_=1./depth;
  cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, active_pt->level_);

  // cam_couple_->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_.x(),uv_.y() );
  // cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, active_pt->level_);
  // float depth;
  // cam_couple_->getD2(active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,depth);
  // invdepth_=1./depth;


}

bool ActivePointProjected::checkOutlier(){
  // float error_intensity = abs(active_pt_->c_ - c_);
  // if ( error_intensity> occlusion_thres_intensity )
  //   return false;
  // float error_gradient = abs(active_pt_->magn_cd_ - magn_cd_);
  // if ( error_gradient> occlusion_thres_gradient )
  //   return false;
  return true;
}


void ActivePoint::updateInvdepthVarAndP(float invdepth, float invdepth_var){
  invdepth_ = invdepth;
  invdepth_var_ = invdepth_var;
  cam_->pointAtDepth(uv_, 1.0/invdepth, p_, p_incamframe_);
}

void ActivePoint::remove(){
  // remove candidate from vector
  removeFromVecByElement(cam_->points_container_->active_points_, this);

  cam_->points_container_->n_active_points_removed_++;

  delete this;
}


PtDataForBA* MarginalizedPoint::initializeDataForBA(){
  PtDataForBA* ptr = new PtDataForBA();
  return ptr;
}

void MarginalizedPoint::remove(){
  // remove candidate from vector
  removeFromVecByElement(cam_->points_container_->marginalized_points_, this);


  delete this;
}
