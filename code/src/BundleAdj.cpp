#include "BundleAdj.h"
#include "LinSystemBA.h"
#include "utils.h"
#include "dso.h"

bool BundleAdj::getMeasurementsInit(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector){
  bool valid = false;
  int n_valid = 0;
  int n_occlusions = 0;

  float total_error = 0;

  // get only errors
  // iterate through all active keyframes
  for( int j=dso_->cameras_container_->keyframes_active_.size()-1; j>=0 ; j--){

    std::shared_ptr<CamCouple> cam_couple = cam_couple_container_->get(0,j);

    // avoid self projections
    if (cam_couple.get()==nullptr)
      continue;

    assert(cam_couple->cam_r_==active_point->cam_);
    assert(cam_couple->cam_m_==dso_->cameras_container_->keyframes_active_[j]);

    MeasBA* measurement = new MeasBA(active_point, cam_couple);
    if(measurement->occlusion_){

      n_occlusions++;
    }
    else if(measurement->valid_){
    // if(measurement->valid_){
      total_error+=abs(measurement->error);
      valid=true;
      measurement_vector->push_back(measurement);
      n_valid++;
    }

  }


  // evaluate occlusion ratio
  float occlusion_valid_ratio = (float)n_occlusions/(float)(n_valid+n_occlusions);
  // evaluate non valid proj ratio
  float valid_ratio = (float)n_valid/((float)dso_->cameras_container_->keyframes_active_.size()-1);

  bool non_valid = false;
  bool occlusion = false;

  // std::cout << active_point->cost_threshold_*0.2 << std::endl;
  // if( (total_error/n_valid) > (active_point->cost_threshold_) ){

  // if( (total_error/n_valid) >total_error_thresh){
  //   // n_points_occlusions_++;
  //   // occlusion=true;
  //   // active_point->remove();
  //   return false;
  // }
  if(occlusion_valid_ratio>occlusion_valid_ratio_thresh){
    // n_points_occlusions_++;
    // occlusion=true;
    return false;
  }
  if(valid_ratio<valid_ratio_thresh){
    // n_points_non_valid_++;
    // non_valid=true;
    return false;
  }

  // if(occlusion){
  // // if(occlusion || non_valid){
  //   // clear vector
  //   for( MeasBA* measurement : *measurement_vector ){
  //     delete measurement;
  //   }
  //   n_points_removed_++;
  //   active_point->remove();
  //   return false;
  // }
  return true;
}


bool BundleAdj::getMeasurements(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector){

  bool measurements_valid = getMeasurementsInit( active_point, i,  measurement_vector);
  if(!measurements_valid)
    return false;
  // load jacobians
  // iterate through all active keyframes
  for( MeasBA* measurement : *measurement_vector){
    measurement->loadJacobians( active_point );
  }


  return true;
}

void BundleAdj::setCamData(){
  int count =0;

  // iterate through keyframes
  for(int i=0; i<dso_->cameras_container_->keyframes_active_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->keyframes_active_[i];
    if(!keyframe->fixed_){
      keyframe->cam_data_for_ba_->c_idx_=count;
      count++;
    }
  }
}

void MarginalizationHandler::setCamIdxs(){
  // iterate through keyframes with prior
  for(int i=0; i<keyframes_with_priors_.size(); i++){
    keyframes_with_priors_[i]->cam_data_for_ba_->c_marg_idx_=i;
  }
}

void MarginalizationHandler::removeCamFromHtilde(int idx){
  unsigned int size_old = H_tilde_.rows();
  unsigned int size_new = H_tilde_.rows()-J_SZ;
  assert(H_tilde_.rows() == H_tilde_.cols());

  if( idx < size_new ){
    int idx_end = idx+J_SZ;
    H_tilde_.block(idx,0,size_old-idx_end,size_old) = H_tilde_.block(idx_end,0,size_old-idx_end,size_old);
    H_tilde_.block(0,idx,size_old,size_old-idx_end) = H_tilde_.block(0,idx_end,size_old,size_old-idx_end);
  }

  H_tilde_.conservativeResize(size_new,size_new);

}

void MarginalizationHandler::addCamToHtilde(){

  assert(H_tilde_.rows() == H_tilde_.cols());
  int size = H_tilde_.rows();

  H_tilde_.conservativeResizeLike(Eigen::MatrixXf::Zero(H_tilde_.rows()+J_SZ,H_tilde_.rows()+J_SZ));

  assert(size+J_SZ==H_tilde_.rows());


}

void MarginalizationHandler::removeCamFromBtilde(int idx){
  unsigned int size_old = b_tilde_.rows();
  unsigned int size_new = b_tilde_.rows()-J_SZ;

  if( idx < size_new ){
    int idx_end = idx+J_SZ;
    b_tilde_.segment(idx,size_old-idx_end) = b_tilde_.segment(idx_end,size_old-idx_end);
  }
  b_tilde_.conservativeResize(size_new);

}

void MarginalizationHandler::addCamToBtilde(){
  int size = b_tilde_.rows();

  b_tilde_.conservativeResizeLike(Eigen::VectorXf::Zero(J_SZ+size));

  assert(size+J_SZ==b_tilde_.size());

}


void MarginalizationHandler::removeKeyframeWithPriors(CameraForMapping* keyframe){

  // get idx
  int idx = getIndex(keyframes_with_priors_ , keyframe);
  removeFromVecByIdx(keyframes_with_priors_, idx);

  // remove col/row in H tilde
  removeCamFromHtilde( idx*J_SZ );

  // remove element in b tilde
  removeCamFromBtilde( idx*J_SZ );
  // .

}

void MarginalizationHandler::addKeyframeWithPriors(CameraForMapping* keyframe){

  // add to keyframe with priors vec
  keyframes_with_priors_.push_back(keyframe);

  // append col/row in H tilde
  addCamToHtilde();

  // add element in b tilde
  addCamToBtilde();


}



void BundleAdj::marginalizeKeyframe(CameraForMapping* keyframe){
  // set flag
  keyframe->marginalized_=true;
  dso_->cameras_container_->moveKeyframeMarginalized(keyframe);
  if(keyframe->cam_data_for_ba_->has_prior_){

    marginalization_handler_->removeKeyframeWithPriors(keyframe);

  }
}

bool BundleAdj::createPrior(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple){

  assert(active_point->cam_==cam_couple->cam_r_);

  PriorMeas* prior_meas = new PriorMeas(active_point,cam_couple);
  if(prior_meas->valid_ && !prior_meas->occlusion_){

    marginalization_handler_->prior_measurements_.push_back(prior_meas);

    prior_meas->p_idx_=n_points_marginalized_;

    return true;
  }
  return false;

}



bool BundleAdj::marginalizePoint(ActivePoint* active_point, CamCoupleContainer* cam_couple_container){


  // remove active point from vector
  removeFromVecByElement(active_point->cam_->points_container_->active_points_, active_point);


  if(!do_marginalization){
    delete active_point;
    return true;
  }

  // create and push marginalized point
  MarginalizedPoint* marg_pt = new MarginalizedPoint(active_point);
  active_point->cam_->points_container_->marginalized_points_.push_back(marg_pt);


  bool valid = false;

  // crete prior
  // iterate through all active keyframes
  for (int i=0; i<cam_couple_container->cam_couple_mat_[0].size(); i++){
    std::shared_ptr<CamCouple> cam_couple = cam_couple_container->get(0,i);
    assert(cam_couple->cam_r_ == active_point->cam_);

    if(cam_couple->cam_m_==active_point->cam_)
      continue;
    if(cam_couple->cam_m_->fixed_)
      continue;

    bool valid_prior = createPrior(active_point, cam_couple);
    if(valid_prior){
      valid = true;

      // if cam m doesn't have priors yet
      if( cam_couple->cam_m_->cam_data_for_ba_->has_prior_==false   ){
        // cam m has prior
        cam_couple->cam_m_->cam_data_for_ba_->has_prior_=true;
        // add keyframe with priors
        marginalization_handler_->addKeyframeWithPriors(cam_couple->cam_m_);
        // fix linearization point
        cam_couple->cam_m_->cam_data_for_ba_->frame_camera_wrt_world_0_=*(cam_couple->cam_m_->frame_camera_wrt_world_);
        cam_couple->cam_m_->cam_data_for_ba_->frame_world_wrt_camera_0_=*(cam_couple->cam_m_->frame_world_wrt_camera_);
        cam_couple->cam_m_->cam_data_for_ba_->a_exposure_0_=cam_couple->cam_m_->a_exposure_;
        cam_couple->cam_m_->cam_data_for_ba_->b_exposure_0_=cam_couple->cam_m_->b_exposure_;
      }

    }
  }

  if(valid){
    n_points_marginalized_++;
    // save linearization point of camera
  }


  delete active_point;

  return valid;

}

void BundleAdj::marginalize(){

  dso_->points_handler_->projectActivePointsOnLastFrame();

  if(debug_optimization && dso_->frame_current_idx_>=debug_start_frame){
    // dso_->points_handler_->projectActivePointsOnLastFrame();
    // dso_->points_handler_->showProjectedActivePoints("before marg");
  }

  // marginalize points and keyframes
  marginalizePointsAndKeyframes();

  // upload priors
  marginalization_handler_->setCamIdxs();
  marginalization_handler_->loadPriorsInLinSys();
  marginalization_handler_->uploadHBTilde();


}


void BundleAdj::updateBMarg(LinSysBA& lin_sys_ba){
  Eigen::VectorXf dx_c = lin_sys_ba.dx_c;
  Eigen::VectorXf dx_c_marg;
  dx_c_marg.resize(marginalization_handler_->b_tilde_.size());

  // iterate through keyframes with priors
  for( int i=0; i<marginalization_handler_->keyframes_with_priors_.size(); i++ ){
    CameraForMapping* cam_r = marginalization_handler_->keyframes_with_priors_[i];
    bool no_r = cam_r->fixed_ || cam_r->marginalized_;
    int c_marg_idx_r = cam_r->cam_data_for_ba_->c_marg_idx_*J_SZ;
    int c_idx_r = cam_r->cam_data_for_ba_->c_idx_*J_SZ;
    if(no_r){
      dx_c_marg.segment<J_SZ>(c_marg_idx_r).setZero();
    }
    else{

      dx_c_marg.segment<J_SZ>(c_marg_idx_r) = dx_c.segment<J_SZ>(c_idx_r);
    }



  }

  // update
  marginalization_handler_->b_tilde_ += marginalization_handler_->H_tilde_*dx_c_marg;


}

void BundleAdj::updateState(LinSysBA& lin_sys_ba, bool only_pts){

  // lin_sys_ba.visualizeH();

  lin_sys_ba.H_cc_ = lin_sys_ba.H_cc_.selfadjointView<Eigen::Upper>();
  // get inverse of schur -> (H_cc_ - H_cp_ H_pp_inv H_pc_)_inv
  // Eigen::VectorXf H_pp_damped = lin_sys_ba.H_pp_ + damp_point_invdepth * Eigen::VectorXf::Ones(lin_sys_ba.H_pp_.size());
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic> H_pp_inv = invDiagonalMatrix(H_pp_damped );

  lin_sys_ba.H_cc_.diagonal() += damp_cam * Eigen::VectorXf::Ones(lin_sys_ba.H_cc_.diagonal().size());
  lin_sys_ba.H_pp_ += damp_point_invdepth * Eigen::VectorXf::Ones(lin_sys_ba.H_pp_.size());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic> H_pp_inv = invDiagonalMatrix(lin_sys_ba.H_pp_);
  Eigen::MatrixXf H_pc_ = lin_sys_ba.H_cp_.transpose();

  // use schur
  if(!only_pts){
    Eigen::MatrixXf Schur = lin_sys_ba.H_cc_ - lin_sys_ba.H_cp_ * H_pp_inv * H_pc_;
    Eigen::MatrixXf Schur_inv = pinvDense( Schur );
    lin_sys_ba.dx_c = Schur_inv * ( -lin_sys_ba.b_c_ + lin_sys_ba.H_cp_ * H_pp_inv * lin_sys_ba.b_p_);
    lin_sys_ba.dx_p = H_pp_inv * ( -lin_sys_ba.b_p_ - H_pc_ * lin_sys_ba.dx_c );

    // lin_sys_ba.dx_c = pinvDense(lin_sys_ba.H_cc_) * ( -lin_sys_ba.b_c_ );

  }
  // no schur
  else{
    // lin_sys_ba.dx_c = pinvDense(lin_sys_ba.H_cc_) * ( -lin_sys_ba.b_c_ );
    lin_sys_ba.dx_p = H_pp_inv * ( -lin_sys_ba.b_p_ );
  }


  // lin_sys_ba.dx_c.setZero();
  lin_sys_ba.dx_p.setZero();

  if(debug_optimization && dso_->frame_current_idx_>=debug_start_frame){
    // dso_->points_handler_->projectActivePointsOnLastFrame();
    // dso_->points_handler_->showProjectedActivePoints("active pts proj during tracking");
    dso_->spectator_->renderState();
    dso_->spectator_->showSpectator();
    std::cout << "chi " << lin_sys_ba.chi << std::endl;
    for ( CameraForMapping* cam : dso_->cameras_container_->keyframes_active_){
      std::cout << cam->name_ << ", a " << cam->a_exposure_ << "; ";
    }
    std::cout << "" << std::endl;
    // std::cout << "dc " << lin_sys_ba.dx_c << std::endl;
    // std::cout << "dp " << lin_sys_ba.dx_p << std::endl;
  }

  lin_sys_ba.updateCameras();
  lin_sys_ba.updatePoints();

  // lin_sys_ba.visualizeH();

  updateBMarg(lin_sys_ba);


}

void BundleAdj::marginalizePointsAndKeyframes(){

  n_points_marginalized_=0;

  int n_cams_marg = 0;

  CameraForMapping* curr_frame = dso_->frame_current_;



  // iterate through keyframes to be marginalized
  for (CameraForMapping* keyframe_to_marg : dso_->cameras_container_->keyframes_to_marginalize_){

    if(marg_pts_in_marg_kf){

      // create cam couple vector
      CamCoupleContainer* cam_couple_container = new CamCoupleContainer(dso_,keyframe_to_marg);
      // marginalize all active points
      for(int i=keyframe_to_marg->points_container_->active_points_.size()-1; i>=0; i--){
        ActivePoint* active_pt = keyframe_to_marg->points_container_->active_points_[i];
        marginalizePoint(active_pt, cam_couple_container);
      }
      delete cam_couple_container;

    }

    // marginalize keyframe
    marginalizeKeyframe(keyframe_to_marg);

  }

  // ***************************************************************************************

  // iterate through frames with active points
  for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->frames_with_active_pts_[i];

    // if(cam_r->points_container_->active_points_.empty()){
    //   dso_->cameras_container_->removeFrameWithActPts(cam_r);
    //   continue;
    // }

    delete cam_couple_container_;
    cam_couple_container_ = new CamCoupleContainer(dso_,cam_r);



    for( int j=cam_r->points_container_->active_points_.size()-1; j>=0; j-- ){
      ActivePoint* active_pt = cam_r->points_container_->active_points_[j];
      active_pt->p_idx_=-1;

      std::vector<MeasBA*>* measurement_vector = new std::vector<MeasBA*>;
      bool measurements_are_valid = getMeasurementsInit( active_pt,i, measurement_vector);
      if(!measurements_are_valid){
        // n_points_occlusions_++;
        // occlusion=true;
        if(active_pt->new_){
          active_pt->remove();
          n_points_removed_++;
        }
        else{
          // std::cout << "APPPPPP" << std::endl;
          marginalizePoint(active_pt, cam_couple_container_);
        }
      }
      deletePointersVec(*measurement_vector);
      delete measurement_vector;
    }
  }



  // ***************************************************************************************

  // marginalize points not in last cam
  // iterate through all keyframe with active points
  for (int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size(); i++){
    CameraForMapping* keyframe = dso_->cameras_container_->frames_with_active_pts_[i];
    // std::shared_ptr<CamCouple> cam_couple_curr_frame = new CamCouple(keyframe,curr_frame);

    CamCoupleContainer* cam_couple_container = new CamCoupleContainer(dso_,keyframe);

    // iterate along all active points
    for(int i=keyframe->points_container_->active_points_.size()-1; i>=0; i--){
      ActivePoint* active_pt = keyframe->points_container_->active_points_[i];

      Eigen::Vector2f uv_curr;
      std::shared_ptr<CamCouple> cam_couple = cam_couple_container->get(0,dso_->cameras_container_->keyframes_active_.size()-1);
      bool valid = cam_couple->getUv( active_pt->uv_.x(),active_pt->uv_.y(),1./active_pt->invdepth_,uv_curr.x(),uv_curr.y() );
      if(!valid)
        continue;

      bool uv_in_range = keyframe->uvInRange(uv_curr);

      MeasBA measurement(active_pt, cam_couple );

      assert(cam_couple->cam_m_==dso_->cameras_container_->keyframes_active_.back());
      if(!uv_in_range || measurement.occlusion_){
        if(measurement.occlusion_)
        if(active_pt->new_){
          active_pt->remove();
          n_points_removed_++;
        }
        else{
          marginalizePoint(active_pt, cam_couple_container);
        }
      }
      else{
        active_pt->new_=false;
      }

    }

    delete cam_couple_container;
    // delete cam_couple_curr_frame;
  }

  // resize new linear system
  n_cams_marg = marginalization_handler_->keyframes_with_priors_.size()*J_SZ;
  marginalization_handler_->lin_sys_increment_->resize(n_cams_marg, n_points_marginalized_);
  assert(marginalization_handler_->H_tilde_.rows()==marginalization_handler_->lin_sys_increment_->H_cc_.rows());

  // remove frames without active points
  for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->frames_with_active_pts_[i];
    if(cam_r->points_container_->active_points_.empty() && cam_r->marginalized_){
      dso_->cameras_container_->removeFrameWithActPts(cam_r);
      cam_r->cam_free_mem();
    }
  }
}

void BundleAdj::optimize(bool only_pts){

  double t_start=getTime();

  // prepare cams data for bundle adjustment
  // setCamData();

  // create linear system
  LinSysBA lin_sys_ba(dso_);

  n_points_non_valid_=0;
  n_points_occlusions_=0;
  n_points_removed_=0;

  // iterations of bundle adjustment
  for(int iteration=0; iteration<max_iterations_ba; iteration++){

    // cam_couple_container_->init();
    int num_points = 0;

    std::vector<std::vector<MeasBA*>*> measurement_vec_vec;

    // iterate through frames with active points
    for( int i=0; i<dso_->cameras_container_->frames_with_active_pts_.size() ; i++){
      CameraForMapping* cam_r = dso_->cameras_container_->frames_with_active_pts_[i];

      // if(cam_r->points_container_->active_points_.empty()){
      //   dso_->cameras_container_->removeFrameWithActPts(cam_r);
      //   continue;
      // }

      delete cam_couple_container_;
      cam_couple_container_ = new CamCoupleContainer(dso_,cam_r, true);


      // for( ActivePoint* active_pt : cam_r->points_container_->active_points_ ){
      for( int j=cam_r->points_container_->active_points_.size()-1; j>=0; j-- ){
        ActivePoint* active_pt = cam_r->points_container_->active_points_[j];
        active_pt->p_idx_=-1;

        std::vector<MeasBA*>* measurement_vector = new std::vector<MeasBA*>;

        bool measurements_valid = getMeasurements(active_pt, i, measurement_vector);
        if(!measurements_valid)
          continue;

        active_pt->p_idx_=num_points;
        num_points++;
        measurement_vec_vec.push_back(measurement_vector);


      }
    }

    assert(num_points>0);
    lin_sys_ba.reinitWithNewPoints(num_points);

    lin_sys_ba.buildLinearSystem(measurement_vec_vec, marginalization_handler_);
    updateState(lin_sys_ba, only_pts);

    // deletePointersVec(measurement_vec_vec);

    // if(debug_optimization){
    //   dso_->points_handler_->projectActivePointsOnLastFrame();
    //   dso_->points_handler_->showProjectedActivePoints("active pts proj during tracking");
    //   dso_->spectator_->renderState();
    //   dso_->spectator_->showSpectator();
    // }

  }

  double t_end=getTime();
  int deltaTime = (t_end-t_start);
  sharedCoutDebug("   - Bundle adjustment: " + std::to_string(deltaTime) + " ms");
  // sharedCoutDebug("       - N points removed: " + std::to_string(n_points_removed_));
  // sharedCoutDebug("           - Occlusions: " + std::to_string(n_points_occlusions_));
  // sharedCoutDebug("           - Non valid: " + std::to_string(n_points_non_valid_));
  // sharedCoutDebug("       - N points marginalized: " + std::to_string(n_points_marginalized_));




}

float MarginalizationHandler::loadPriorInLinSys(PriorMeas* prior_meas){

  float weight = prior_meas->getWeight();

  int m_idx = prior_meas->cam_couple_->cam_m_->cam_data_for_ba_->c_marg_idx_*J_SZ;
  int p_idx = prior_meas->p_idx_;

  assert(m_idx < lin_sys_increment_->c_size_);
  assert(p_idx < lin_sys_increment_->p_size_);

  lin_sys_increment_->H_cc_.block<J_SZ,J_SZ>(m_idx,m_idx).triangularView<Eigen::Upper>() += prior_meas->J_m_transpose*weight*prior_meas->J_m;

  lin_sys_increment_->H_cp_.block<J_SZ,1>(m_idx,p_idx) += prior_meas->J_m_transpose*(weight*prior_meas->J_d);


  lin_sys_increment_->H_pp_(p_idx) += prior_meas->J_d*weight*prior_meas->J_d;

  lin_sys_increment_->b_c_.segment<J_SZ>(m_idx) += prior_meas->J_m_transpose*(weight*prior_meas->error);

  lin_sys_increment_->b_p_(p_idx) += prior_meas->J_d*weight*prior_meas->error;

  return weight*prior_meas->error*prior_meas->error;
}

void MarginalizationHandler::loadPriorsInLinSys(){

  float chi = 0;
  // iterate through all priors
  for(PriorMeas* prior_meas : prior_measurements_){
    chi += loadPriorInLinSys(prior_meas);
    delete prior_meas;
  }

  // std::cout << "chi marg: " << chi/prior_measurements_.size() << std::endl;

  prior_measurements_.clear();

}

void MarginalizationHandler::uploadHBTilde(){

  // get inverse of schur -> (H_cc_ - H_cp_ H_pp_inv H_pc_)_inv
  Eigen::VectorXf H_pp_damped = lin_sys_increment_->H_pp_ + damp_point_invdepth * Eigen::VectorXf::Ones(lin_sys_increment_->H_pp_.size());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic> H_pp_inv = invDiagonalMatrix(H_pp_damped );
  lin_sys_increment_->H_cc_ = lin_sys_increment_->H_cc_.selfadjointView<Eigen::Upper>();
  Eigen::MatrixXf H_pc = lin_sys_increment_->H_cp_.transpose();

  // lin_sys_increment_->visualizeH();

  // use schur
  Eigen::MatrixXf H_tilde_increment = (lin_sys_increment_->H_cc_ - lin_sys_increment_->H_cp_ * H_pp_inv * H_pc);
  Eigen::VectorXf b_tilde_increment = (lin_sys_increment_->b_c_ - lin_sys_increment_->H_cp_ * H_pp_inv * lin_sys_increment_->b_p_);

  // // no schur
  // Eigen::MatrixXf H_tilde_increment = (lin_sys_increment_->H_cc_ );
  // Eigen::VectorXf b_tilde_increment = (lin_sys_increment_->b_c_);

  H_tilde_ += H_tilde_increment;
  b_tilde_ += b_tilde_increment;
  // H_tilde_ = H_tilde_increment;
  // b_tilde_ = b_tilde_increment;
}
