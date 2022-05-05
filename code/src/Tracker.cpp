#include "Tracker.h"
#include "CameraForMapping.h"
#include "dso.h"
#include "CamCouple.h"
#include "LinSystemTracking.h"
#include "utils.h"


void Tracker::trackCam(bool groundtruth){
  if(groundtruth){
    dso_->frame_current_->setGrountruthPose();
    sharedCoutDebug("   - Frame tracked (groundtruth)");
  }
  else{
    trackCam();
  }
}

void Tracker::setInitialGuess(){

  switch (dso_->parameters_->guess_type)
  {
    case POSE_CONSTANT:{
      dso_->frame_current_->assignPose( *(dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_) );
      break;}
    case VELOCITY_CONSTANT:{
      Eigen::Isometry3f transf = (*dso_->cameras_container_->getThirdLastFrame()->frame_world_wrt_camera_)*(*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_);
      Eigen::Isometry3f pose = (*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_)*transf;
      dso_->frame_current_->assignPose( pose );
      break;}
    case PERS_GUESS:{
      Eigen::Isometry3f transf = (*dso_->cameras_container_->getThirdLastFrame()->frame_world_wrt_camera_)*(*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_);
      transf.translation()*=0.5;
      transf.linear().setIdentity();
      Eigen::Isometry3f pose = (*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_)*transf;
      dso_->frame_current_->assignPose( pose );
      break;}

    }

  // if(==POSE_CONSTANT){
  //   dso_->frame_current_->assignPose( *(dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_) );
  // }
  // else if(dso_->parameters_->guess_type==VELOCITY_CONSTANT){
  //   Eigen::Isometry3f transf = (*dso_->cameras_container_->getThirdLastFrame()->frame_world_wrt_camera_)*(*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_);
  //   Eigen::Isometry3f pose = (*dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_)*transf;
  //   dso_->frame_current_->assignPose( pose );
  // }
  // else{
  //   throw std::invalid_argument( "initial guess type has wrong value" );
  // }
}


bool Tracker::checkConvergence(float chi){

  bool out = false;
  if(chi_history_.size()>0){
    if ( abs(chi_history_.back()-chi) < dso_->parameters_->conv_threshold ){
      out = true;
    }
  }

  chi_history_.push_back(chi);
  return out;
}


bool Tracker::chiUpdateAndCheck(float chi){

  bool out = false;
  if(chi_history_.size()>1){
    if ( (chi_history_.back()-chi) < (chi_history_[0]-chi_history_[1])*dso_->parameters_->stop_threshold ){
      out = true;
    }
  }

  chi_history_.push_back(chi);
  return out;
}

void Tracker::showProjectedActivePoints(int level, CamCoupleContainer& cam_couple_container){

  // Image<colorRGB>* show_img( dso_->frame_current_->pyramid_->getC(level)->returnColoredImgFromIntensityImg( "Tracking debug, level: "+std::to_string(level) ) );
  Image<colorRGB>* show_img( dso_->frame_current_->pyramid_->getC(level)->returnColoredImgFromIntensityImg( "Tracking debug" ) );

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
    CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[i];

    // get coarse active points to project
    std::vector<ActivePoint*>& act_pts_coarse = cam_r->points_container_->active_points_;

    // for each act pt coarse
    for( ActivePoint* active_pt_coarse : act_pts_coarse){

      ActivePointProjected* act_pt_proj = new ActivePointProjected(active_pt_coarse,cam_couple_container.get(i,0));
      if(level){
        Eigen::Vector2f uv; pxl pixel;
        cam_couple_container.get(i,0)->getUv( active_pt_coarse->uv_.x(),active_pt_coarse->uv_.y(),1./active_pt_coarse->invdepth_,uv.x(),uv.y() );
        cam_couple_container.get(i,0)->cam_m_->uv2pixelCoords( uv, pixel, level );
        colorRGB color = cam_couple_container.get(i,0)->cam_m_->cam_parameters_->invdepthToRgb(act_pt_proj->invdepth_);
        show_img->setPixel( pixel,color);
      }
      else{

        dso_->frame_current_->points_container_->drawPoint(act_pt_proj, show_img, !(level));
      }
    }
  }

  show_img->show(pow(2,level));
  cv::waitKey(0);


}

void Tracker::trackCam(){

  double deltaTime_tot = 0;

  setInitialGuess();

  // create cam couple container
  CamCoupleContainer cam_couple_container(dso_,ALL_KFS_ON_LAST);

  // create linear system
  LinSysTracking lin_sys_tracking(dso_);

  // iterate through levels
  for (int level=dso_->parameters_->coarsest_level-1; level>=0 ; level--){
  // for (int level=0; level<1 ; level++){

    for(int iteration=0; iteration<dso_->parameters_->max_iterations_ls; iteration++){
    // while(true){
      double t_start=getTime();

      int n_meas =0;


      // iterate through keyframes (except last)
      for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
        CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[i];

        // select active points vector
        // std::vector<ActivePoint*>& active_points = cam_r->points_container_->getActivePoints(level);
        std::vector<ActivePoint*>& active_points = cam_r->points_container_->active_points_;

        // iterate through active points (at current coarse level)
        for (int j=0; j<active_points.size(); j++){
          ActivePoint* active_point = active_points[j];

          // get measurement
          MeasTracking measurement(MeasTracking(active_point, cam_couple_container.get(i,0), level, dso_ ));

          if(measurement.valid_){
            // update linear system with that measurement
            lin_sys_tracking.addMeasurement(measurement);
            n_meas++;
          }

        }

      }

      assert(n_meas>0);
      lin_sys_tracking.updateCameraPose();


      double t_end=getTime();
      deltaTime_tot+=(t_end-t_start);

      // if(dso_->parameters_->debug_tracking || dso_->frame_current_idx_>50){
      if(dso_->parameters_->debug_tracking ){

        // std::cout << "level " << level << std::endl;
        // std::cout << "level " << level << ", chi " << lin_sys_tracking.chi << std::endl;

        dso_->points_handler_->projectActivePointsOnLastFrame();
        // dso_->points_handler_->showProjectedActivePoints("active pts proj during tracking");
        showProjectedActivePoints(level, cam_couple_container);
        dso_->spectator_->renderState();
        dso_->spectator_->showSpectator();

      }
      cam_couple_container.update();

      bool stop = false;

      if (checkConvergence(lin_sys_tracking.chi))
        break;


      lin_sys_tracking.clear();

      if(stop){
        break;
      }
    }

    chi_history_.clear();
  }
  dso_->spectator_->renderState();
  dso_->spectator_->showSpectator();
  sharedCoutDebug("   - Frame tracked: " + std::to_string((int)deltaTime_tot) + " ms");

}
