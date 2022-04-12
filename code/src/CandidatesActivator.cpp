#include "CandidatesActivator.h"
#include "CameraForMapping.h"
#include "dso.h"

void ActptpresenceMat::clear(){
  for(int i=0; i<nrows_; i++){
    for(int j=0; j<ncols_; j++){
      mat_[i][j]=false;
    }
  }

}

void ActptpresenceMat::buildLevel0(std::shared_ptr<std::vector<std::shared_ptr<ActivePointProjected>>> v){
  for(std::shared_ptr<ActivePointProjected> active_pt_proj : *v){
    pxl pixel = active_pt_proj->pixel_/2;
    int row = (int)(pixel.y()-0.5);
    int col = (int)(pixel.x()-0.5);
    assert(row>0 && col>0 && row<nrows_ && col<ncols_);
    mat_[row][col]=true;
  }
}

void ActptpresenceMat::updateFromLowerLevel(std::shared_ptr<ActptpresenceMat> lower){
  assert(nrows_ == lower->nrows_/2);
  assert(ncols_ == lower->ncols_/2);

  for( int i = 0; i < lower->nrows_; i+=2 ) {
    for( int j = 0; j < lower->ncols_; j+=2 ) {
      bool val = (lower->mat_[i][j] || lower->mat_[i+1][j] || lower->mat_[i][j+1] || lower->mat_[i+1][j+1]);
      mat_[i/2][j/2] = val;
    }
  }
}

bool RegionsMat::checkRegion(int row, int col){

  assert(row>0 && row<nrows_ && col>0 && col<ncols_);
  bool isnotnull = mat_[row][col]!=nullptr;
  return isnotnull;
}


std::shared_ptr<RegionsMat> RegionsMat::getHigherLevel(){
  int nrows_new = nrows_/2;
  int ncols_new = ncols_/2;
  std::shared_ptr<RegionsMat> mat_new (new RegionsMat(nrows_new, ncols_new) );

  return mat_new;
}

void ActptpresenceMatVec::clear(){
  updated_=false;
  for(std::shared_ptr<ActptpresenceMat> actptpresencemat : actptpresencemat_vec_){
    actptpresencemat->clear();
  }

}

void RegMatVec::clear(){
  regionsmat_vec_.clear();

}

void RegVecMat::init(){
  for (int i = 0 ; i < cand_activator_->dso_->parameters_->reg_level ; i++) {
    regs_[i].resize(3);
  }
}


void RegVecMat::clear(){
  updated_=false;
  for (int i = 0 ; i < regs_.size() ; i++) {
    for (int j = 0 ; j < regs_[i].size() ; j++) {
      regs_[i][j].clear();
    }
  }

}

void RegVecMat::update(){
  if(updated_)
    clear();

  for(std::shared_ptr<CandidateProjected> cand_proj : *(cand_activator_->dso_->frame_current_->points_container_->candidates_projected_) ){
    for(int level=0; level<cand_activator_->dso_->parameters_->reg_level; level++){
      pxl pixel = cand_proj->pixel_/pow(2,level+1);
      int row = (int)(pixel.y()-0.5);
      int col = (int)(pixel.x()-0.5);
      std::shared_ptr<RegionsMat> regmat =cand_activator_->regmat_vec_.regionsmat_vec_[level];

      bool regexists = regmat->checkRegion(row,col);
      if(!regexists){
        
      }
    }
  }
  updated_=true;
}

void ActptpresenceMatVec::update(){

  if(updated_==true)
    clear();

  for(int i=0; i<actptpresencemat_vec_.size(); i++){
    std::shared_ptr<ActptpresenceMat> actptpresencemat = actptpresencemat_vec_[i];
    if(i==0){
      actptpresencemat->buildLevel0( dso_->frame_current_->points_container_->active_points_projected_ );
    }
    else{
      actptpresencemat->updateFromLowerLevel(actptpresencemat_vec_[i-1]);
    }
  }
  updated_=true;
}


void CandidatesActivator::update(){
  // actptpresencemat_vec_.clear();
  // regmat_vec_->clear();
  // regvec_mat_.clear();
  regmat_vec_.clear();

  actptpresencemat_vec_.update();
  regvec_mat_.update();
}

// void CandidatesActivator::activateCandidates(){
//   collectRegions();
// }
//
// void CandidatesActivator::collectRegions(){
//   // iterate through region levels
//   for(int lev=0; lev<dso_->parameters_->reg_level_; lev++){
//     collectRegion(lev);
//   }
// }
//
// void CandidatesActivator::collectRegion(int level){
//   int grid_size_x = dso_->cam_parameters_->resolution_x /pow(2,level+1);
//   int grid_size_y = dso_->cam_parameters_->resolution_y /pow(2,level+1);
//   std::shared_ptr<Region> arr[ grid_size_x ][ grid_size_y ];
//
//   std::vector<std::shared_ptr<Region>> regions_forbid;
//   std::vector<std::shared_ptr<Region>> regions_activate;
//   if(level==0){
//     std::vector<std::shared_ptr<ActivePointProjected>> v_forbid = dso_->frame_current_->points_container_->active_points_projected_;
//     std::vector<std::shared_ptr<CandidateProjected>> v_activate = dso_->frame_current_->points_container_->candidates_projected_;
//     // for each active_pt_projected
//     for(std::shared_ptr<CandidateProjected> cand_proj : v_activate ){
//       int x = cand_proj->pixel_.x();
//       int y = cand_proj->pixel_.y();
//       arr[x][y] = Region(false);  // writing to (x, y)
//     }
//   }
//   else{
//
//   }
// }
