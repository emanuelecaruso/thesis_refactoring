#include "CandidatesActivator.h"
#include "CameraForMapping.h"
#include "utils.h"
#include "dso.h"

void Region::pushCandidateProj(CandidateProjected* cand_proj){
  auto it_ = std::find (candidates_projected_.begin(), candidates_projected_.end(), cand_proj);
  assert(it_ == candidates_projected_.end());

  auto lb_cmp = [](CandidateProjected* const & x, float d) -> bool
    { return x->cand_->invdepth_var_ < d; };

  auto it = std::lower_bound(candidates_projected_.begin(), candidates_projected_.end(), cand_proj->cand_->invdepth_var_, lb_cmp);
  candidates_projected_.insert ( it , cand_proj );

  // candidates_projected_.push_back(cand_proj);
}

void Region::pushSubreg(Region* subreg){
  subregions_.push_back(subreg);
  subreg->parent_=this;
}

void CandprojpresenceMat::init(Dso* dso){
  nrows_=dso->cam_parameters_->resolution_y;
  ncols_=dso->cam_parameters_->resolution_x;
  mat_=new bool*[nrows_];
  for( int i = 0; i < nrows_; i++ ) {
    mat_[i] = new bool[ncols_];
  }
  clear();
}

void CandprojpresenceMat::clear(){

  for( int i = 0; i < nrows_; i++ ) {
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

void ActptpresenceMat::buildLevel0(std::vector<ActivePointProjected*>& v){
  for(ActivePointProjected* active_pt_proj : v){
    pxl pixel = active_pt_proj->pixel_;
    int row = (int)(pixel.y()-0.5);
    int col = (int)(pixel.x()-0.5);
    assert(row>=0 && col>=0 && row<nrows_ && col<ncols_);
    mat_[row][col]=true;
  }
}

void ActptpresenceMat::updateFromLowerLevel(ActptpresenceMat* lower){
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
    for( int j = 0; j < ncols_; j++ ) {
      delete mat_[i][j];
    }
  }

  for( int i = 0; i < nrows_; i++ ) {
    delete mat_[i];
  }

  for( int i = 0; i < nrows_; i++ ) {
    mat_[i] = new Region*[ncols_];
    for( int j = 0; j < ncols_; j++ ) {
      mat_[i][j] = nullptr;
    }
  }
}

bool RegionsMat::checkRegion(int row, int col){

  assert(row>=0 && row<nrows_ && col>=0 && col<ncols_);
  bool isnotnull = mat_[row][col] != nullptr;
  return isnotnull;
}


RegionsMat* RegionsMat::getHigherLevel(){
  int nrows_new = nrows_/2;
  int ncols_new = ncols_/2;
  RegionsMat* mat_new (new RegionsMat(nrows_new, ncols_new) );

  return mat_new;
}

void ActptpresenceMatVec::init(Dso* dso){
  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  actptpresencemat_vec_.resize(reg_level);
  for(int i=0; i<reg_level; i++){
    ActptpresenceMat* ptr(new ActptpresenceMat(res_y/pow(2,i),res_x/pow(2,i)) );
    // actptpresencemat_vec_[i] = std::make_shared<ActptpresenceMat>(res_y/pow(2,i),res_x/pow(2,i));
    actptpresencemat_vec_[i] = ptr;
  }
}

void ActptpresenceMatVec::clear(){
  updated_=false;
  for(ActptpresenceMat* actptpresencemat : actptpresencemat_vec_){
    actptpresencemat->clear();
  }

}

void RegMatVec::init(Dso* dso){

  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  regionsmat_vec_.resize(reg_level);
  for(int i=0; i<reg_level; i++){
    if(i==0){
      RegionsMat* ptr(nullptr);
      regionsmat_vec_[i] = ptr;
    }
    else{
      regionsmat_vec_[i] = new RegionsMat(res_y/pow(2,i),res_x/pow(2,i));
    }
  }
}

void RegMatVec::clear(){
  std::vector<RegionsMat*> regionsmat_vec_;
  for(int i=0; i<regionsmat_vec_.size(); i++){
    regionsmat_vec_[i]->clear();
  }

}

void RegVecMat::init(){
  regs_.resize(reg_level);
  for (int i = 0 ; i < reg_level ; i++) {
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
  // if(updated_)
    clear();

  for(int i=cand_activator_->dso_->frame_current_->points_container_->candidates_projected_.size()-1; i>=0; i--  ){
  // for(CandidateProjected* cand_proj : cand_activator_->dso_->frame_current_->points_container_->candidates_projected_ ){
    CandidateProjected* cand_proj = cand_activator_->dso_->frame_current_->points_container_->candidates_projected_[i];
    Region* subreg;
    bool prev_reg_already_exists = false;

    pxl pixel_prev = cand_proj->pixel_;
    int row_prev = (int)(pixel_prev.y()-0.5);
    int col_prev = (int)(pixel_prev.x()-0.5);

    for(int level=1; level<reg_level; level++){

      ActptpresenceMat* actptpresencemat =cand_activator_->actptpresencemat_vec_.actptpresencemat_vec_[level-1];
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
      RegionsMat* regmat =cand_activator_->regmat_vec_.regionsmat_vec_[level];

      bool regexists = regmat->checkRegion(row,col);
      if(!regexists){
        // get number of forbidden subregions
        int num_forbid_subregs = (int)actptpresencemat->mat_[row*2][col*2] + (int)actptpresencemat->mat_[row*2+1][col*2] + (int)actptpresencemat->mat_[row*2][col*2+1] + (int)actptpresencemat->mat_[row*2+1][col*2+1];
        assert(num_forbid_subregs<4);

        // create new region
        regmat->mat_[row][col] = new Region(level, num_forbid_subregs);
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
    ActptpresenceMat* actptpresencemat = actptpresencemat_vec_[i];
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
  regmat_vec_.clear();
  candprojpresencemat_.clear();
  actptpresencemat_vec_.reset();
  regvec_mat_.reset();
}


void CandidatesActivator::removeCand(CandidateProjected* cand_proj){

  // remove projected candidate from vector
  removeFromVecByElement(cand_proj->cam_->points_container_->candidates_projected_, cand_proj);

  // remove candidate from vector
  removeFromVecByElement(cand_proj->cand_->cam_->points_container_->candidates_, cand_proj->cand_);


}

void CandidatesActivator::activateCandidate(CandidateProjected* cand_proj){
  // remove projected candidate and candidate from vectors
  removeCand(cand_proj);

  // create new active point
  ActivePoint* active_point = new ActivePoint(cand_proj->cand_);

  // push active point
  cand_proj->cand_->cam_->points_container_->active_points_.push_back(active_point);

  // create projected active point
  ActivePointProjected* active_pt_proj (new ActivePointProjected(active_point, cand_proj));

  // push projected active point
  cand_proj->cam_->points_container_->active_points_projected_.push_back(active_pt_proj);


}

void CandidatesActivator::removeEmptyRegion(Region* reg){
  if(reg->candidates_projected_.empty()){
    // pass through subregs

    Region* reg_curr = reg;
    for( int subreg_lev=2; subreg_lev<=reg_level-1; subreg_lev++){

      if(reg_curr->parent_->subregions_.size()<=1){
        removeFromVecByElement(reg_curr->parent_->subregions_, reg_curr);
      }
      else
        break;
      reg_curr = reg_curr->parent_;

    }
  }
}


void CandidatesActivator::activateCandidates(){

  dso_->points_handler_->projectActivePointsOnLastFrame();
  dso_->points_handler_->projectCandidatesOnLastFrame();

  double t_start=getTime();

  reset();

  // printActPresenceMat(actptpresencemat_vec_.actptpresencemat_vec_[0]);
  // printCandPresenceMat();

  int num_candidates_to_activate = max_num_active_points-dso_->frame_current_->points_container_->active_points_projected_.size();
  int num_candidates_to_activate_old = num_candidates_to_activate;

  int num_candidates_activated = 0;
  for(int level=reg_level-1; level>0; level--){
    for(int i=0; i<4; i++){

      while(regvec_mat_.regs_[level][i].size()>0){

        if(num_candidates_to_activate<=0){
          level=0;
          i=4;
          break;
        }

        Region* reg = regvec_mat_.regs_[level][i].back();
        regvec_mat_.regs_[level][i].pop_back();
        assert(reg->candidates_projected_.size()<=4);

        if(level>1){

          // remove subregion
          Region* subreg = reg->subregions_.back();
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
            removeFromVecByIdx(regvec_mat_.regs_[subreg->level_][subreg->num_forbid_subregs_],subreg->idx_regvecmat_);
            subreg=subreg->subregions_.back();
          }

          if(subreg->candidates_projected_.empty())
            continue;
          // activate candidate
          CandidateProjected* cand_proj = subreg->candidates_projected_[0];
          activateCandidate(cand_proj);
          num_candidates_activated++;

          // remove proj cand from subreg
          subreg->candidates_projected_.erase(subreg->candidates_projected_.begin());
          // removeEmptyRegion(subreg);
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

          if(reg->candidates_projected_.empty())
            continue;
          // activate candidate
          CandidateProjected* cand_proj = reg->candidates_projected_.back();
          activateCandidate(cand_proj);
          num_candidates_activated++;

          // remove proj cand from reg
          reg->candidates_projected_.pop_back();
          // removeEmptyRegion(reg);
        }

        // num_current_active_points_++;
        num_candidates_to_activate--;
      }

    }
  }

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size() ; i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    if(!keyframe->has_active_pts_ && !(keyframe->points_container_->active_points_.empty()) )
      dso_->cameras_container_->addFrameWithActPts(keyframe);
  }

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Candidates activation: " + std::to_string(deltaTime)+" ms");
  sharedCoutDebug("       - Candidates activated: "+ std::to_string(num_candidates_activated) );
  sharedCoutDebug("       - N candidates to activate: " + std::to_string(num_candidates_to_activate_old) );
  sharedCoutDebug("       - Current active points: " + std::to_string(dso_->frame_current_->points_container_->active_points_projected_.size()) );

}
void CandidatesActivator::printActPresenceMat(ActptpresenceMat* actptpresencemat){

  Image<float>* new_img = new Image<float>("act presence mat");
  new_img->initImage(actptpresencemat->nrows_, actptpresencemat->ncols_);
  new_img->setAllPixels(1);
  for( int i = 0; i < actptpresencemat->nrows_; i++ ) {
    for( int j = 0; j < actptpresencemat->ncols_; j++ ) {
        if (actptpresencemat->mat_[i][j])
          new_img->setPixel(i,j,0);
    }
  }
  new_img->show(2);
  cv::waitKey(0);
}

void CandidatesActivator::printCandPresenceMat(){

  Image<float>* new_img = new Image<float>("cands mat");
  new_img->initImage(candprojpresencemat_.nrows_, candprojpresencemat_.ncols_);
  new_img->setAllPixels(1);
  for( int i = 0; i < candprojpresencemat_.nrows_; i++ ) {
    for( int j = 0; j < candprojpresencemat_.ncols_; j++ ) {
        if (candprojpresencemat_.mat_[i][j])
          new_img->setPixel(i,j,0);
    }
  }
  new_img->show(2);
  cv::waitKey(0);
}
