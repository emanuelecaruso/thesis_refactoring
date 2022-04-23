#include "CandidatesActivator.h"
#include "CameraForMapping.h"
#include "CoarseRegions.h"
#include "utils.h"
#include "dso.h"

void Region::pushCandidateProj(std::shared_ptr<CandidateProjected> cand_proj){
  candidates_projected_.push_back(cand_proj);
}

void Region::pushSubreg(std::shared_ptr<Region> subreg){
  subregions_.push_back(subreg);
}

void CandprojpresenceMat::init(std::shared_ptr<Dso> dso){
  nrows_=dso->cam_parameters_->resolution_y;
  ncols_=dso->cam_parameters_->resolution_x;
  mat_=new bool*[nrows_];
  for( int i = 0; i < nrows_; i++ ) {
    mat_[i] = new bool[ncols_];
    for( int j = 0; j < ncols_; j++ ) {
        mat_[i][j] = false;
    }
  }
}

void ActptpresenceMat::clear(){
  for(int i=0; i<nrows_; i++){
    for(int j=0; j<ncols_; j++){
      mat_[i][j]=false;
    }
  }

}

void ActptpresenceMat::buildLevel0(std::shared_ptr<std::vector<std::shared_ptr<ActivePointProjected>>> v){
  for(std::shared_ptr<ActivePointProjected> active_pt_proj : *v){
    pxl pixel = active_pt_proj->pixel_;
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

void RegionsMat::clear(){
  for( int i = 0; i < nrows_; i++ ) {
    mat_[i] = new std::shared_ptr<Region>[ncols_];
    for( int j = 0; j < ncols_; j++ ) {
        mat_[i][j] = nullptr;
    }
  }
}

bool RegionsMat::checkRegion(int row, int col){

  assert(row>=0 && row<nrows_ && col>=0 && col<ncols_);
  bool isnotnull = mat_[row][col].get()!=nullptr;
  return isnotnull;
}


std::shared_ptr<RegionsMat> RegionsMat::getHigherLevel(){
  int nrows_new = nrows_/2;
  int ncols_new = ncols_/2;
  std::shared_ptr<RegionsMat> mat_new (new RegionsMat(nrows_new, ncols_new) );

  return mat_new;
}

void ActptpresenceMatVec::init(std::shared_ptr<Dso> dso){
  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  actptpresencemat_vec_.resize(dso->parameters_->reg_level);
  for(int i=0; i<dso->parameters_->reg_level; i++){
    std::shared_ptr<ActptpresenceMat> ptr(new ActptpresenceMat(res_y/pow(2,i),res_x/pow(2,i)) );
    // actptpresencemat_vec_[i] = std::make_shared<ActptpresenceMat>(res_y/pow(2,i),res_x/pow(2,i));
    actptpresencemat_vec_[i] = ptr;
  }
}

void ActptpresenceMatVec::clear(){
  updated_=false;
  for(std::shared_ptr<ActptpresenceMat> actptpresencemat : actptpresencemat_vec_){
    actptpresencemat->clear();
  }

}

void RegMatVec::init(std::shared_ptr<Dso> dso){

  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  regionsmat_vec_.resize(dso->parameters_->reg_level);
  for(int i=0; i<dso->parameters_->reg_level; i++){
    if(i==0){
      std::shared_ptr<RegionsMat> ptr(nullptr);
      regionsmat_vec_[i] = ptr;
    }
    else{
      regionsmat_vec_[i] = std::make_shared<RegionsMat>(res_y/pow(2,i),res_x/pow(2,i));
    }
  }
}

void RegMatVec::clear(){
  std::vector<std::shared_ptr<RegionsMat>> regionsmat_vec_;
  for(int i=0; i<regionsmat_vec_.size(); i++)
    regionsmat_vec_[i]->clear();

}

void RegVecMat::init(){
  regs_.resize(cand_activator_->dso_->parameters_->reg_level);
  for (int i = 0 ; i < cand_activator_->dso_->parameters_->reg_level ; i++) {
    if(i>0)
      regs_[i].resize(4);
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

void RegVecMat::reset(){
  if(updated_)
    clear();

  for(std::shared_ptr<CandidateProjected> cand_proj : *(cand_activator_->dso_->frame_current_->points_container_->candidates_projected_) ){
    std::shared_ptr<Region> subreg;
    bool prev_reg_already_exists = false;

    pxl pixel_prev = cand_proj->pixel_;
    int row_prev = (int)(pixel_prev.y()-0.5);
    int col_prev = (int)(pixel_prev.x()-0.5);

    for(int level=1; level<cand_activator_->dso_->parameters_->reg_level; level++){

      std::shared_ptr<ActptpresenceMat> actptpresencemat =cand_activator_->actptpresencemat_vec_.actptpresencemat_vec_[level-1];
      if(actptpresencemat->mat_[row_prev][col_prev]){
        break;
      }

      if(level==1){
        if(cand_activator_->candprojpresencemat_.mat_[row_prev][col_prev]){
          break;
        }
        else{
          cand_activator_->candprojpresencemat_.mat_[row_prev][col_prev]=true;
        }
      }

      // pxl pixel = cand_proj->pixel_/pow(2,level);
      int row = row_prev/2;
      int col = col_prev/2;
      // int row = (int)(pixel.y()-0.5);
      // int col = (int)(pixel.x()-0.5);
      std::shared_ptr<RegionsMat> regmat =cand_activator_->regmat_vec_.regionsmat_vec_[level];

      bool regexists = regmat->checkRegion(row,col);
      if(!regexists){
        // get number of forbidden subregions
        int num_forbid_subregs = (int)actptpresencemat->mat_[row*2][col*2] + (int)actptpresencemat->mat_[row*2+1][col*2] + (int)actptpresencemat->mat_[row*2][col*2+1] + (int)actptpresencemat->mat_[row*2+1][col*2+1];
        assert(num_forbid_subregs<4);

        // create new region
        regmat->mat_[row][col].reset(new Region(level, num_forbid_subregs));
        // push region in regvecmat
        cand_activator_->regvec_mat_.regs_[level][num_forbid_subregs].push_back(regmat->mat_[row][col]);
        regmat->mat_[row][col]->idx_regvecmat_=cand_activator_->regvec_mat_.regs_[level][num_forbid_subregs].size()-1;
      }
      if(level==1){
        regmat->mat_[row][col]->pushCandidateProj(cand_proj);
      }
      else{
        if(!prev_reg_already_exists){
          regmat->mat_[row][col]->pushSubreg(subreg);
        }
        else{
          assert(regexists);
        }
      }
      if (regmat->mat_[row][col]->num_forbid_subregs_>0){
        break;
      }

      row_prev = row;
      col_prev = col;
      subreg=regmat->mat_[row][col];
      prev_reg_already_exists = regexists;

    }
  }
  updated_=true;
}

void ActptpresenceMatVec::reset(){

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


void CandidatesActivator::reset(){
  // actptpresencemat_vec_.clear();
  // regmat_vec_->clear();
  // regvec_mat_.clear();
  regmat_vec_.clear();

  actptpresencemat_vec_.reset();
  regvec_mat_.reset();
}

// void CandidatesActivator::activateCandidateFromRegion(){
//
// }

void CandidatesActivator::activateCandidate(std::shared_ptr<CandidateProjected> cand_proj){
  // remove projected candidate from vector
  std::shared_ptr<std::vector<std::shared_ptr<CandidateProjected>>> cp = cand_proj->cam_->points_container_->candidates_projected_;
  cp->erase(std::remove(cp->begin(), cp->end(), cand_proj), cp->end());

  // remove candidate from vector
  std::shared_ptr<std::vector<std::shared_ptr<Candidate>>> c = cand_proj->cand_->cam_->points_container_->candidates_;
  c->erase(std::remove(c->begin(), c->end(), cand_proj->cand_), c->end());

  // create new active point
  std::shared_ptr<ActivePoint> active_point (new ActivePoint(cand_proj->cand_));

  // push active point
  cand_proj->cand_->cam_->points_container_->active_points_->push_back(active_point);

  // add active point in coarse region
  active_point->cam_->points_container_->coarse_regions_->addActivePoint(active_point);

  // create projected active point
  std::shared_ptr<ActivePointProjected> active_pt_proj (new ActivePointProjected(active_point, cand_proj));

  // push projected active point
  cand_proj->cam_->points_container_->active_points_projected_->push_back(active_pt_proj);


}


void CandidatesActivator::activateCandidates(){

  double t_start=getTime();

  reset();
  int num_candidates_to_activate = dso_->parameters_->max_num_active_points-num_current_active_points_;
  for(int level=dso_->parameters_->reg_level-1; level>0; level--){
    for(int i=0; i<4; i++){
      while(regvec_mat_.regs_[level][i].size()>0){

        if(num_candidates_to_activate<=0){
          level=0;
          i=4;
          break;
        }

        std::shared_ptr<Region> reg = regvec_mat_.regs_[level][i].back();
        regvec_mat_.regs_[level][i].pop_back();
        assert(reg->candidates_projected_.size()<=4);

        if(level>1){

          // remove subregion
          std::shared_ptr<Region> subreg = reg->subregions_.back();
          reg->subregions_.pop_back();
          // if there are still subregions, push in regvec matrix
          if(reg->subregions_.size()>0){
            reg->num_forbid_subregs_++;
            if(reg->num_forbid_subregs_<4){
              regvec_mat_.regs_[level][reg->num_forbid_subregs_].push_back(reg);
              reg->idx_regvecmat_=regvec_mat_.regs_[level][reg->num_forbid_subregs_].size()-1;
            }
          }
          // pass through subregs
          for( int subreg_lev=subreg->level_; subreg_lev>1; subreg_lev--){
            regvec_mat_.regs_[subreg->level_][subreg->num_forbid_subregs_].erase(regvec_mat_.regs_[subreg->level_][subreg->num_forbid_subregs_].begin()+subreg->idx_regvecmat_);
            subreg=subreg->subregions_.back();
          }
          // activate candidate
          std::shared_ptr<CandidateProjected> cand_proj = subreg->candidates_projected_.back();
          activateCandidate(cand_proj);
        }
        else{
          if(reg->candidates_projected_.size()>0){
            assert(i<=4);

            reg->num_forbid_subregs_++;
            if(reg->num_forbid_subregs_<4){
              regvec_mat_.regs_[level][reg->num_forbid_subregs_].push_back(reg);
              reg->idx_regvecmat_=regvec_mat_.regs_[level][reg->num_forbid_subregs_].size()-1;
            }
          }
          // activate candidate
          std::shared_ptr<CandidateProjected> cand_proj = reg->candidates_projected_.back();
          activateCandidate(cand_proj);
        }

        num_current_active_points_++;
        num_candidates_to_activate--;

      }

    }
  }

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Candidate activation: "+ std::to_string(deltaTime)+" ms");

}
