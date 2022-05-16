#include "CandidatesActivator.h"
#include "CameraForMapping.h"
#include "utils.h"
#include "dso.h"


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
    int row = (int)trunc(pixel.y());
    int col = (int)trunc(pixel.x());
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

  // delete
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



void ActptpresenceMatVec::init(Dso* dso){
  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  actptpresencemat_vec_.resize(reg_level);
  for(int i=0; i<reg_level; i++){
    ActptpresenceMat* ptr(new ActptpresenceMat( trunc(res_y/pow(2,i)) , trunc(res_x/pow(2,i)) ) );
    actptpresencemat_vec_[i] = ptr;
  }
}

void ActptpresenceMatVec::clear(){

  for(ActptpresenceMat* actptpresencemat : actptpresencemat_vec_){
    actptpresencemat->clear();
  }

}

void RegMatVec::init(Dso* dso){

  int res_x = dso->cam_parameters_->resolution_x;
  int res_y = dso->cam_parameters_->resolution_y;
  regionsmat_vec_.resize(reg_level);
  for(int i=0; i<reg_level; i++){
    regionsmat_vec_[i] = new RegionsMat( trunc(res_y/pow(2,i)) , trunc(res_x/pow(2,i)) );
  }
}

void RegMatVec::clear(){
  for(int i=0; i<regionsmat_vec_.size(); i++){
    regionsmat_vec_[i]->clear();
  }

}

void RegVecMat::init(){
  regs_.resize(reg_level);
  for (int i = 0 ; i < reg_level ; i++) {
    if(i==0)
      regs_[i].resize(1);
    else
      regs_[i].resize(4);
  }
}


void RegVecMat::clear(){

  for (int i = 0 ; i < regs_.size() ; i++) {
    for (int j = 0 ; j < regs_[i].size() ; j++) {
      regs_[i][j].clear();
    }
  }

}


void ActptpresenceMatVec::reset(){

  for(int i=0; i<actptpresencemat_vec_.size(); i++){
    ActptpresenceMat* actptpresencemat = actptpresencemat_vec_[i];
    if(i==0){
      actptpresencemat->buildLevel0( dso_->frame_current_->points_container_->active_points_projected_ );
    }
    else{
      actptpresencemat->updateFromLowerLevel(actptpresencemat_vec_[i-1]);
    }
  }

}



void CandidatesActivator::removeCand(CandidateProjected* cand_proj){

  // remove projected candidate from vector
  removeFromVecByElement(cand_proj->cam_->points_container_->candidates_projected_, cand_proj);

  // remove candidate from vector
  removeFromVecByElement(cand_proj->cand_->cam_->points_container_->candidates_, cand_proj->cand_);


}


Region* CandidatesActivator::updateRegion(Region* reg_old){

  assert(reg_old!=nullptr);

  // if there is an active point on region old, return false
  if(actptpresencemat_vec_.actptpresencemat_vec_[reg_old->level_]->mat_[reg_old->row_][reg_old->col_])
    return nullptr;


  int row_new = (int)trunc(reg_old->row_/2);
  int col_new = (int)trunc(reg_old->col_/2);
  int level_new = reg_old->level_+1;


  assert(row_new < regmat_vec_.regionsmat_vec_[level_new]->nrows_);
  assert(col_new < regmat_vec_.regionsmat_vec_[level_new]->ncols_);
  assert(row_new >= 0);
  assert(col_new >= 0);
  Region*& reg = regmat_vec_.regionsmat_vec_[level_new]->mat_[row_new][col_new];

  // if region doesn't exists yet
  if(reg==nullptr){
    // create region
    reg = new Region(reg_old);
    // count num of subregs with active pts
    int n_act_pts = 0;
    n_act_pts += actptpresencemat_vec_.actptpresencemat_vec_[reg_old->level_]->mat_[row_new*2][col_new*2];
    n_act_pts += actptpresencemat_vec_.actptpresencemat_vec_[reg_old->level_]->mat_[row_new*2+1][col_new*2];
    n_act_pts += actptpresencemat_vec_.actptpresencemat_vec_[reg_old->level_]->mat_[row_new*2][col_new*2+1];
    n_act_pts += actptpresencemat_vec_.actptpresencemat_vec_[reg_old->level_]->mat_[row_new*2+1][col_new*2+1];
    assert(n_act_pts<4);
    // insert in regvec_mat_
    if(reg->row_==196 && reg->col_==241)
      std::cout << "AOOOOOO " << level_new << " " << n_act_pts << " " << reg << std::endl;
    insertRegInVec(reg, regvec_mat_.regs_[level_new][n_act_pts] );
    reg->num_forbid_subregs_=n_act_pts;

    // update region at higher level
    if(level_new<reg_level-1){
      Region* reg_new = updateRegion(reg);
      reg->parent_=reg_new;
      if(reg_new!=nullptr){
        assert(checkElementInVec(reg_new->subregions_, reg));
        assert(checkElementInVec(reg->parent_->subregions_, reg));
      }
    }
  }
  // otherwise
  else{
    // check if subregion is already present
    bool subreg_exists = checkElementInVec( reg->subregions_, reg_old);
    // otherwise
    if(!subreg_exists){
      // insert subregion
      insertRegInVec(reg_old, reg->subregions_ );
    }

    if( reg_old->var_ < reg->var_ ){
      reg->var_ = reg_old->var_;
    }
  }


  return reg;
}

bool CandidatesActivator::updateLeafRegion(CandidateProjected* cand_proj){

  int row_new = (int)(trunc(cand_proj->pixel_.y()));
  int col_new = (int)(trunc(cand_proj->pixel_.x()));

  // if there is an active point on same region, return false
  if(actptpresencemat_vec_.actptpresencemat_vec_[0]->mat_[row_new][col_new])
    return false;

  Region*& reg = regmat_vec_.regionsmat_vec_[0]->mat_[row_new][col_new];

  // if region doesn't exists yet
  if(reg==nullptr){
    // create region
    reg = new Region(cand_proj);
    // push reg in regvec_mat_
    regvec_mat_.regs_[0][0].push_back(reg);

    // update region at higher level
    Region* reg_new = updateRegion(reg);
    reg->parent_=reg_new;
    assert(checkElementInVec(reg_new->subregions_, reg));
    assert(checkElementInVec(reg->parent_->subregions_, reg));
  }
  // otherwise
  else{
    if( cand_proj->cand_->invdepth_var_ < reg->var_ ){
      reg->cand_proj_ = cand_proj;
      reg->var_ = cand_proj->cand_->invdepth_var_;
    }
  }


}

void CandidatesActivator::rebuild(){

  // iterate through all candidates proj
  for(int i=dso_->frame_current_->points_container_->candidates_projected_.size()-1; i>=0; i--  ){
    CandidateProjected* cand_proj = dso_->frame_current_->points_container_->candidates_projected_[i];
    updateLeafRegion(cand_proj);

  }

}

void CandidatesActivator::clearAll(){
  actptpresencemat_vec_.clear();
  regmat_vec_.clear();
  regvec_mat_.clear();
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

void CandidatesActivator::clearParentIfEmpty( Region* reg){

  if(reg->cand_proj_!=nullptr)
  std::cout << reg->cand_proj_->cam_->name_ << std::endl;

  assert(!reg->removed_);

  std::cout << "clear " << reg->level_  << " " << reg->num_forbid_subregs_  <<  " " << reg << " " << reg->row_ <<" "<< reg->col_ << " " << reg->subregions_.empty() << std::endl;
  if(reg->subregions_.empty()){
    // remove from regvecmat
    assert(checkElementInVec(regvec_mat_.regs_[reg->level_][reg->num_forbid_subregs_], reg));
    removeFromVecByElement(regvec_mat_.regs_[reg->level_][reg->num_forbid_subregs_], reg);
    assert(!checkElementInVec(regvec_mat_.regs_[reg->level_][reg->num_forbid_subregs_], reg));

    if( reg->parent_!=nullptr ){

      // std::cout << "clear proj to reg " << reg << " " << reg->subregions_.size() << " " << reg->level_ << std::endl;
      reg->removed_=true;

      // TODO
      // if current region has no active points yet
      if(!reg->has_active_pts_){
        // it will have an active point
        reg->has_active_pts_=true;
        // the parent have to be shifted
        if( j_<regvec_mat_.regs_[i_].size()-1 && !reg->subregions_.empty()){
          insertRegInVec( reg->parent_, regvec_mat_.regs_[reg->parent_->level_][reg->parent_->num_forbid_subregs_+1] );
          reg->parent_->num_forbid_subregs_+=1;
        }
      }

      // std::cout << "porco " << std::endl;
      // remove from parent
      assert(checkElementInVec(reg->parent_->subregions_, reg));
      removeFromVecByElement(reg->parent_->subregions_, reg);
      assert(!checkElementInVec(reg->parent_->subregions_, reg));

      // std::cout << "dio " << std::endl;



      // std::cout << "cane " << std::endl;

      clearParentIfEmpty( reg->parent_ );

    }
  }
}


CandidateProjected* CandidatesActivator::getCandProjFromReg( Region* reg ){

  std::cout << "reg to proj " << reg << " " << reg->subregions_.size() << " " << reg->level_ << std::endl;

  if(reg->cand_proj_==nullptr){
    assert(!reg->subregions_.empty());

    return getCandProjFromReg( reg->subregions_.front() );
  }
  else{

    // clear from parent if there are no more subregions
    clearParentIfEmpty( reg );
    return reg->cand_proj_;
  }
}

CandidateProjected* CandidatesActivator::getNextCandProj(){


  for(; i_>=0; i_--){
    for(; j_<regvec_mat_.regs_[i_].size(); j_++){

      std::vector<Region*>& v = regvec_mat_.regs_[i_][j_];
      std::cout << i_ << " " << j_ << " " << regvec_mat_.regs_[i_][j_].size() << std::endl;
      if(v.empty()){
        break;
      }
      Region* reg = v.front();
      // v.erase(v.begin());

      // from reg to cand proj
      CandidateProjected* cand_proj = getCandProjFromReg( reg );

      // if(i_>0 && j_<regvec_mat_.regs_[i_].size()-1 && !reg->subregions_.empty()){
      //   insertRegInVec( reg, regvec_mat_.regs_[i_][j_+1] );
      //   reg->num_forbid_subregs_+=1;
      // }
      assert(cand_proj != nullptr);

      return cand_proj;

    }
    j_=0;
  }
  return nullptr;
}



void CandidatesActivator::insertRegInVec(Region* reg, std::vector<Region*>& v ){

  auto lb_cmp = [](Region* const & x, float var) -> bool
    { return x->var_ < var; };

  auto it = std::lower_bound(v.begin(), v.end(), reg->var_, lb_cmp);
  v.insert ( it , reg );
  assert(checkElementInVec(v,reg));


}



void CandidatesActivator::activateCandidates(){

  dso_->points_handler_->projectActivePointsOnLastFrame();
  dso_->points_handler_->projectCandidatesOnLastFrame();

  double t_start=getTime();

  clearAll();
  actptpresencemat_vec_.reset(); // update actpt regions
  // for( ActptpresenceMat* a : actptpresencemat_vec_.actptpresencemat_vec_)
  //   printActPresenceMat(a);

  rebuild();  // update all regions

  // for( RegionsMat* r : regmat_vec_.regionsmat_vec_)
  //   printRegionsMat(r);

  int num_candidates_available=dso_->frame_current_->points_container_->candidates_projected_.size();
  int num_candidates_activated=0;
  int num_candidates_to_activate = max_num_active_points-dso_->frame_current_->points_container_->active_points_projected_.size();

  i_=regvec_mat_.regs_.size()-1;
  j_=0;

  for(int i=0; i<num_candidates_to_activate; i++){
    // std::cout << i << "/" << num_candidates_to_activate << std::endl;
    CandidateProjected* cand_proj = getNextCandProj();
    if( cand_proj == nullptr )
      break;

    activateCandidate(cand_proj);
    num_candidates_activated++;

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
  sharedCoutDebug("       - N candidates to activate: " + std::to_string(num_candidates_to_activate) );
  sharedCoutDebug("       - N candidates avilable: " + std::to_string(num_candidates_available) );
  sharedCoutDebug("       - Candidates activated: "+ std::to_string(num_candidates_activated) );
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

void CandidatesActivator::printRegionsMat(RegionsMat* regionsmat){

  Image<float>* new_img = new Image<float>("regions mat");
  new_img->initImage(regionsmat->nrows_, regionsmat->ncols_);
  new_img->setAllPixels(1);
  for( int i = 0; i < regionsmat->nrows_; i++ ) {
    for( int j = 0; j < regionsmat->ncols_; j++ ) {
      if( regionsmat->mat_[i][j]==nullptr )
        continue;

      if (regionsmat->mat_[i][j]->subregions_.size()>0 || regionsmat->mat_[i][j]->cand_proj_!=nullptr ){
        new_img->setPixel(i,j,0);
      }
    }
  }
  new_img->show(2);
  cv::waitKey(0);
}
