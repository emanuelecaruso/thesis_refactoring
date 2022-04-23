#include "PointsContainer.h"
#include "CamCouple.h"
#include "CoarseRegions.h"
#include <algorithm>    // std::max


void Candidate::showCandidate(){
  cam_->pyramid_->getC(level_)->showImgWithColoredPixel( pixel_, pow(2,level_), "cand");
}

std::shared_ptr<CoarseRegions> PointsContainer::initCoarseRegions(){
   std::shared_ptr<CoarseRegions> coarse_regions(new CoarseRegions(this, parameters_->coarsest_level-1));
   return coarse_regions;
}

void PointsContainer::drawPoint(std::shared_ptr<Point> point, std::shared_ptr<Image<colorRGB>> show_img){
  pxl pixel= point->pixel_;
  colorRGB color = black;
  if( point->invdepth_>0 )
    color = cam_->cam_parameters_->invdepthToRgb(point->invdepth_);
  show_img->drawCircle( color, pixel, 1, 2);
}

void PointsContainer::showCandidates(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(candidates_->size())+" candidates";
  std::shared_ptr<Image<colorRGB>> show_img( cam_->pyramid_->getC(parameters_->candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<candidates_->size(); i++){
    std::shared_ptr<Candidate> candidate = candidates_->at(i);
    drawPoint(candidate,show_img);
  }

  show_img->show(pow(2,parameters_->candidate_level));
  cv::waitKey(0);

}

void PointsContainer::showProjectedCandidates(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(candidates_projected_->size())+" projected candidates";
  std::shared_ptr<Image<colorRGB>> show_img( cam_->pyramid_->getC(parameters_->candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<candidates_projected_->size(); i++){
    std::shared_ptr<CandidateProjected> candidate_proj = candidates_projected_->at(i);
    drawPoint(candidate_proj,show_img);
  }

  show_img->show(pow(2,parameters_->candidate_level));
  cv::waitKey(0);

}



void PointsContainer::showActivePoints(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(active_points_->size())+" active points";
  std::shared_ptr<Image<colorRGB>> show_img( cam_->pyramid_->getC(parameters_->candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<active_points_->size(); i++){
    std::shared_ptr<ActivePoint> active_pt = active_points_->at(i);
    drawPoint(active_pt,show_img);
  }

  show_img->show(pow(2,parameters_->candidate_level));
  cv::waitKey(0);

}

void PointsContainer::showCoarseActivePoints(int level){
  coarse_regions_->showCoarseLevel(level);


}

void PointsContainer::showProjectedActivePoints(){
  double alpha = 1;

  std::string name = cam_->name_+" , "+std::to_string(active_points_projected_->size())+" projected active points";
  std::shared_ptr<Image<colorRGB>> show_img( cam_->pyramid_->getC(parameters_->candidate_level)->returnColoredImgFromIntensityImg(name) );

  // iterate through candidates
  for(int i=0; i<active_points_projected_->size(); i++){
    std::shared_ptr<ActivePointProjected> active_pt_proj = active_points_projected_->at(i);
    drawPoint(active_pt_proj,show_img);
  }

  show_img->show(pow(2,parameters_->candidate_level));
  cv::waitKey(0);

}

std::vector<std::shared_ptr<ActivePoint>>& PointsContainer::getActivePoints(){
  return getActivePoints(0);
}

std::vector<std::shared_ptr<ActivePoint>>& PointsContainer::getActivePoints(int level){
  assert(level>=0 && level<parameters_->coarsest_level);
  if(level==0){
    std::vector<std::shared_ptr<ActivePoint>>& out = *active_points_;
    return out;
  }
  else{
    return (coarse_regions_->getCoarseActivePoints(level));

  }
}


void CandidateProjected::init(std::shared_ptr<Candidate> cand, std::shared_ptr<CamCouple> cam_couple_){
  cam_couple_->getUv( cand->uv_.x(),cand->uv_.y(),1./cand->invdepth_,uv_.x(),uv_.y() );
  cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, cand->level_);
  cam_=cam_couple_->cam_m_;
  level_=cand->level_;
  float depth;
  cam_couple_->getD2(cand->uv_.x(),cand->uv_.y(),1./cand->invdepth_,depth);
  invdepth_=1./depth;
}


void ActivePointProjected::init(std::shared_ptr<ActivePoint> active_pt, std::shared_ptr<CamCouple> cam_couple_){
  cam_couple_->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_.x(),uv_.y() );
  cam_couple_->cam_m_->uv2pixelCoords( uv_, pixel_, active_pt->level_);
  float depth;
  cam_couple_->getD2(active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,depth);
  invdepth_=1./depth;
}

void ActivePoint::updateInvdepthVarAndP(float invdepth, float invdepth_var){
  invdepth_ = invdepth;
  invdepth_var_ = invdepth_var;
  cam_->pointAtDepth(uv_, 1.0/invdepth, p_, p_incamframe_);
}
