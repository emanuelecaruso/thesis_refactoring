#include "Tracker.h"
#include "CameraForMapping.h"
#include "dso.h"
#include "CamCouple.h"
#include "CoarseRegions.h"
#include "LinSystem.h"


void Tracker::trackCam(bool groundtruth){
  if(groundtruth){
    dso_->frame_current_->setGrountruthPose();
    sharedCoutDebug("   - Frame tracked (groundtruth)");
  }
  else{
    trackCam();
    sharedCoutDebug("   - Frame tracked");
  }
}

void Tracker::setInitialGuess(){
  if(dso_->parameters_->guess_type==POSE_CONSTANT){
    dso_->frame_current_->assignPose( *(dso_->cameras_container_->getSecondLastFrame()->frame_camera_wrt_world_) );
  }
  else if(dso_->parameters_->guess_type==VELOCITY_CONSTANT){

  }
  else{
    throw std::invalid_argument( "initial guess type has wrong value" );
  }
}

bool Tracker::chiUpdateAndCheck(float chi){
  bool out = true;
  if(chi_history_.size()>1){
    if ((chi_history_.back()-chi) < (chi_history_[0]-chi_history_[1])*0.1 )
      out = false;
  }

  chi_history_.push_back(chi);
  return out;
}

void Tracker::showProjectedActivePoints(int level, std::shared_ptr<CamCoupleContainer> cam_couple_container){

  std::shared_ptr<Image<colorRGB>> show_img( dso_->frame_current_->pyramid_->getC(level)->returnColoredImgFromIntensityImg("PORCODDIOINFAME") );

  // iterate through keyframes (except last)
  for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
    std::shared_ptr<CameraForMapping> cam_r = dso_->cameras_container_->keyframes_active_[i];

    // get coarse active points to project
    std::vector<std::shared_ptr<ActivePoint>>& act_pts_coarse = cam_r->points_container_->getActivePoints(level);

    // for each act pt coarse
    for( std::shared_ptr<ActivePoint> active_pt_coarse : act_pts_coarse){
      // project it
      std::shared_ptr<ActivePointProjected> act_pt_proj(new ActivePointProjected(active_pt_coarse,cam_couple_container->get(i,0)));
      dso_->frame_current_->points_container_->drawPoint(act_pt_proj, show_img, !(level));
    }
  }

  show_img->show(pow(2,level));
  cv::waitKey(0);


}

void Tracker::trackCam(){

  setInitialGuess();

  std::shared_ptr<CamCoupleContainer> cam_couple_container(new CamCoupleContainer(dso_,ALL_KFS_ON_LAST));
  std::shared_ptr<LinSysTracking> lin_sys_tracking( new LinSysTracking(dso_));

  // iterate through levels
  for (int level=dso_->parameters_->coarsest_level-1; level>=0 ; level--){
  // for (int level=0; level<1 ; level++){

    for(int iteration=0; iteration<dso_->parameters_->max_iterations_ls; iteration++){
    // while(true){
      int n_meas =0;

      // iterate through keyframes (except last)
      for( int i=0; i<dso_->cameras_container_->keyframes_active_.size()-1 ; i++){
        std::shared_ptr<CameraForMapping> cam_r = dso_->cameras_container_->keyframes_active_[i];

        // select active points vector
        std::vector<std::shared_ptr<ActivePoint>>& active_points = cam_r->points_container_->getActivePoints(level);

        // iterate through active points (at current coarse level)
        for (int j=0; j<active_points.size(); j++){
          std::shared_ptr<ActivePoint> active_point = active_points[j];

          // get measurement
          std::shared_ptr<MeasTracking> measurement(new MeasTracking(active_point, cam_couple_container->get(i,0) ) );
          assert(cam_couple_container->get(i,0)->cam_r_==cam_r && cam_couple_container->get(i,0)->cam_m_==dso_->frame_current_);
          if(measurement->valid){
            // update linear system with that measurement
            lin_sys_tracking->addMeasurement(measurement);
            n_meas++;
          }

        }
        // if(dso_->parameters_->debug_tracking){
        //   std::cout << "it " << iteration << std::endl;
        //   std::cout << "num meas " << n_meas << std::endl;
        //   std::cout << "cam r " << cam_couple_container->get(i,0)->cam_r_->name_ << ", cam m " << cam_couple_container->get(i,0)->cam_m_->name_ << std::endl;
        // }
      }

      lin_sys_tracking->updateCameraPose();
      cam_couple_container->update();
      if(dso_->parameters_->debug_tracking){

        // std::cout << "level " << level << std::endl;
        // std::cout << "chi " << lin_sys_tracking->chi << std::endl;

        dso_->points_handler_->projectActivePointsOnLastFrame();
        // dso_->points_handler_->showProjectedActivePoints("active pts proj during tracking");
        showProjectedActivePoints(level, cam_couple_container);
        dso_->spectator_->renderState();
        dso_->spectator_->showSpectator();

      }
      lin_sys_tracking->clear();

      bool stop = chiUpdateAndCheck(lin_sys_tracking->chi);
      if(stop)
        continue;
    }

  }
  chi_history_.clear();

}
