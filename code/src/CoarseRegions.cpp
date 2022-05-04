#include "CoarseRegions.h"
#include "CameraForMapping.h"
#include "utils.h"
#include "dso.h"

void CoarseRegion::updateFromActivePts(){
  float sum=0;
  float normalizer=0;
  for(ActivePoint* active_pt : active_pts_){
    sum+=active_pt->invdepth_/active_pt->invdepth_var_;
    normalizer+=1.0/active_pt->invdepth_var_;
  }
  invdepth_=sum/normalizer;
  invdepth_var_=active_pts_.size()/normalizer;
}

void CoarseRegion::updateFromSubregions(){
  float sum=0;
  float normalizer=0;
  for(CoarseRegion* subreg : subregions_){
    sum+=subreg->invdepth_/subreg->invdepth_var_;
    normalizer+=1.0/subreg->invdepth_var_;
  }
  invdepth_=sum/normalizer;
  invdepth_var_=subregions_.size()/normalizer;
}

ActivePoint* CoarseRegion::generateCoarseActivePoint(){
  ActivePoint* active_pt = new ActivePoint(cam_, pixel_, level_+1, invdepth_, invdepth_var_);
  return active_pt;
}



void CoarseRegionMatVec::init(PointsContainer* points_container, int levels){
  int rows = points_container->cam_->cam_parameters_->resolution_y/2;
  int cols = points_container->cam_->cam_parameters_->resolution_x/2;
  for(int i=0; i<levels; i++){
    CoarseRegionMat* coarse_region_mat(new CoarseRegionMat(rows,cols) );
    coarseregionsmat_vec_.push_back(coarse_region_mat);
    rows/=2;
    cols/=2;

  }
}

void CoarseRegions::getLevelRowCol(ActivePoint* active_pt, int level, int& row, int& col){

  row = (int)(active_pt->pixel_.y()-0.5)/pow(2,level+1);
  col = (int)(active_pt->pixel_.x()-0.5)/pow(2,level+1);

}


void CoarseRegions::addActivePoint(ActivePoint* active_pt){

  int row, col;
  getLevelRowCol(active_pt, 0, row, col);
  pxl pixel(col,row);
  CoarseRegion* prev_coarse_reg = coarseregionsmat_vec_.coarseregionsmat_vec_[0]->mat_[row][col];
  if(prev_coarse_reg == nullptr){
    prev_coarse_reg = new CoarseRegion(points_container_->cam_, pixel, 0);
    coarseregions_vec_[0].push_back(prev_coarse_reg);
  }

  prev_coarse_reg->active_pts_.push_back(active_pt);
  for(int level=1; level<levels_; level++){
    row/=2;
    col/=2;
    pixel.x()=col;
    pixel.y()=row;
    CoarseRegion* curr_coarse_reg = coarseregionsmat_vec_.coarseregionsmat_vec_[level]->mat_[row][col];
    if(curr_coarse_reg == nullptr){
      curr_coarse_reg = new CoarseRegion(points_container_->cam_, pixel, level);
      coarseregions_vec_[level].push_back(curr_coarse_reg);
    }
    curr_coarse_reg->subregions_.push_back(prev_coarse_reg);
    prev_coarse_reg->parent_=curr_coarse_reg;
    prev_coarse_reg=curr_coarse_reg;
  }
}

void CoarseRegions::removeFromParent(CoarseRegion* coarse_reg){
  if(coarse_reg->subregions_.empty() && coarse_reg->active_pts_.empty()){

    if(coarse_reg->parent_ != nullptr){

      std::vector<CoarseRegion*>& v = coarse_reg->parent_->subregions_;
      v.erase(std::remove(v.begin(), v.end(), coarse_reg), v.end());
      removeFromParent(coarse_reg->parent_);

      v=coarseregions_vec_[coarse_reg->level_];
      v.erase(std::remove(v.begin(), v.end(), coarse_reg), v.end());


    }

    CoarseRegion* ptr(nullptr);
    coarse_reg=ptr;
  }
}


void CoarseRegions::removeActivePoint(ActivePoint* active_pt){
  int row, col;
  getLevelRowCol(active_pt, 0, row, col);
  CoarseRegion* coarse_reg = coarseregionsmat_vec_.coarseregionsmat_vec_[0]->mat_[row][col];
  assert(coarse_reg != nullptr);

  std::vector<ActivePoint*>& v = coarse_reg->active_pts_;
  assert(v.size()>0);

  int v_size = v.size();
  v.erase(std::remove(v.begin(), v.end(), active_pt), v.end());
  assert(v_size!=v.size());

  removeFromParent(coarse_reg);


}

void CoarseRegions::generateCoarseActivePoints(){
  actptscoarse_vec_.clear();

  for(int level=0; level<levels_; level++){

    std::vector<CoarseRegion*>& v = coarseregions_vec_[level];
    for(CoarseRegion* coarse_reg : v){
      if(level==0){
        coarse_reg->updateFromActivePts();
      }
      else{
        coarse_reg->updateFromSubregions();
      }
      ActivePoint* active_pt = coarse_reg->generateCoarseActivePoint();
      assert(active_pt->level_==level+1);
      actptscoarse_vec_.push(active_pt);
    }

  }
}

std::vector<ActivePoint*>& CoarseRegions::getCoarseActivePoints(int level){
  assert(level>0 && level<points_container_->parameters_->coarsest_level);
  for(ActivePoint* act_pt : actptscoarse_vec_.vec_[level-1]){
    assert(act_pt->level_==level);
  }

  return actptscoarse_vec_.vec_[level-1];
}


void CoarseRegions::showCoarseLevel(int level){

  assert(level>0);
  std::vector<ActivePoint*>& vector_of_coarse_active_points = actptscoarse_vec_.vec_[level-1];

  double alpha = 1;

  std::string name = points_container_->cam_->name_+" , "+std::to_string(vector_of_coarse_active_points.size())+" coarse active points, level: "+std::to_string(level);
  Image<colorRGB>* show_img( points_container_->cam_->pyramid_->getC(level)->returnColoredImgFromIntensityImg(name) );

  // iterate through active points
  for(int i=0; i<vector_of_coarse_active_points.size(); i++){
    ActivePoint* coarse_active_pt = vector_of_coarse_active_points.at(i);
    points_container_->drawPoint(coarse_active_pt,show_img, false);
  }

  show_img->show(pow(2,level));
  cv::waitKey(0);
}
