#include "spectator.h"
#include "dso.h"
#include <thread>
#include <vector>
#include <mutex>

CamParameters* Spectator::initCamParams(){
  CamParameters* spectator_params (new CamParameters(dso_->cam_parameters_));
  spectator_params->resolution_x*=spec_upscaling;
  spectator_params->resolution_y*=spec_upscaling;
  spectator_params->pixel_width/=spec_upscaling;
  spectator_params->pixel_meter_ratio*spec_upscaling;

  return spectator_params;
}

Camera* Spectator::initCam(){
  Camera* spectator_cam (new Camera("Spectator", spectator_params_));
  spectator_cam->frame_camera_wrt_world_ = new Eigen::Isometry3f;
  spectator_cam->frame_world_wrt_camera_ = new Eigen::Isometry3f;
  return spectator_cam;
}

Image<colorRGB>* Spectator::initImage(){
  Image<colorRGB>* spectator_image( new Image<colorRGB> ("Spectator"));
  spectator_image->initImage(spectator_params_->resolution_y, spectator_params_->resolution_x);
  spectator_image->setAllPixels(background_color_);
  return spectator_image;
}

void Spectator::spectateDso(){
  // dso_->waitForKeyframeAdded();
  dso_->waitForNewFrame();
  showSpectator();

  while(true){

    renderState();
    showSpectator();

  }
}

void Spectator::renderState(){

  reinitSpectator();
  renderPoints();
  renderCamsAndKFs();

}

void Spectator::showSpectator(int i){
  spectator_image_->show(1);
  // if(!(debug_optimization))
  //   waitkey(1);
  cv::waitKey(i);

}

void Spectator::reinitSpectator(){
  // std::lock_guard<std::mutex> locker(mu_cout);
  spectator_image_->setAllPixels(background_color_);
  Eigen::Isometry3f pose = getSpectatorPose();
  spectator_cam_->assignPose(pose);
}

void Spectator::renderPoints(){
  // iterate through all cameras
  int num_kfs = dso_->cameras_container_->frames_.size()-1;
  for ( int i = 0; i<num_kfs; i++){
    CameraForMapping* cam = dso_->cameras_container_->frames_[i];
    if (cam->keyframe_){
      // render marginalized points
      int num_marg_pts = cam->points_container_->marginalized_points_.size();
      for ( int j = num_marg_pts-1; j>=0; j--){
        MarginalizedPoint* marg_pt = cam->points_container_->marginalized_points_[j];
        plotPt(marg_pt, black);
      }
    }
  }
  for ( int i = 0; i<num_kfs; i++){
    CameraForMapping* cam = dso_->cameras_container_->frames_[i];
    if (cam->keyframe_){
      // render active points
      int num_act_pts = cam->points_container_->active_points_.size();
      for ( int j = num_act_pts-1; j>=0; j--){
        ActivePoint* act_pt = cam->points_container_->active_points_.at(j);

        plotPt(act_pt, magenta);
      }
    }
  }

}

void Spectator::renderCamsAndKFs(){
  // iterate through all cameras
  int num_kfs = dso_->frame_current_idx_+1;
  for ( int i = 0; i<num_kfs; i++){
    CameraForMapping* cam = dso_->cameras_container_->frames_[i];

    if (cam->grountruth_camera_->frame_camera_wrt_world_!=nullptr){
      // plotCam(cam->grountruth_camera_, orange);
      plotCam(cam->grountruth_camera_, black);
    }

    if (cam->keyframe_){
      std::vector<CameraForMapping*>& v = dso_->cameras_container_->keyframes_active_;

      if (std::count(v.begin(), v.end(), cam)) {
        plotCam(cam, red);
      }
      else{
        plotCam(cam, magenta);
      }
    }
    else if (!cam->discarded_during_initialization){
      plotCam(cam, blue);
    }

  }
}


Eigen::Isometry3f Spectator::getSpectatorPose(){
  // get first active keyframe
  CameraForMapping* first_keyframe = dso_->frame_current_;
  // spec wrt first keyframe
  Eigen::Isometry3f kf1_T_spec;
  kf1_T_spec.linear().setIdentity();
  kf1_T_spec.translation() << 0,0,-spec_distance;

  Eigen::Isometry3f pose =  ( first_keyframe->access_frame_camera_wrt_world() )*kf1_T_spec;

  return pose;

}

bool Spectator::plotPt(MarginalizedPoint* pt, const colorRGB& color){
  bool success = plotPt( pt->p_, color);
  return success;
}

bool Spectator::plotPt(ActivePoint* pt, const colorRGB& color){
  bool success = plotPt( pt->p_, color);
  return success;
}

bool Spectator::plotPt(Eigen::Vector3f& pt, const colorRGB& color){
  pxl pixel;
  bool success = plotPt( pt, color, pixel);
  return success;
}

bool Spectator::plotPt(ActivePoint* pt, const colorRGB& color, pxl& pixel){
  bool success = plotPt( pt->p_, color, pixel);
  return success;
}

bool Spectator::plotPt(MarginalizedPoint* pt, const colorRGB& color, pxl& pixel){
  bool success = plotPt( pt->p_, color, pixel);
  return success;
}

bool Spectator::plotPt(Eigen::Vector3f& pt, const colorRGB& color, pxl& pixel){
  Eigen::Vector2f uv;
  float depth;
  bool pt_in_front = spectator_cam_->projectPoint(pt,uv,depth);
  if(!pt_in_front)
    return false;

  spectator_cam_->uv2pixelCoords(uv, pixel);

  if(spectator_image_->pixelInRange(pixel)){
    spectator_image_->drawCircle(  spectator_cam_->cam_parameters_->invdepthToRgb( 1.0/(depth-spec_distance*1) ) , pixel,1,2);
    // spectator_image_->drawCircle(  spectator_cam_->cam_parameters_->depthToRgb( depth-spec_distance ) , pixel,1,2);
    // spectator_image_->drawCircle( color, pixel,1,2);
    return true;
  }
  return false;

}

bool Spectator::plotLine(pxl& pixel1, pxl& pixel2, const colorRGB& color ){

  if(spectator_image_->pixelInRange(pixel1) || spectator_image_->pixelInRange(pixel2) ){
    spectator_image_->drawLine( color, pixel1, pixel2 );
    return true;
  }
  return false;
}

bool Spectator::plotCam(Camera* cam, const colorRGB& color){

  // Eigen::Isometry3f T;
  Eigen::Isometry3f T = cam->access_frame_camera_wrt_world();
  Eigen::Matrix3f R = T.linear();
  Eigen::Vector3f p = T.translation();
  float delta_d = rendered_cams_size;
  float delta_y = rendered_cams_size/2;
  float delta_x = delta_y*spectator_params_->aspect;
  Eigen::Vector3f p_tr(delta_x,delta_y,delta_d);
  Eigen::Vector3f p_tl(-delta_x,delta_y,delta_d);
  Eigen::Vector3f p_br(delta_x,-delta_y,delta_d);
  Eigen::Vector3f p_bl(-delta_x,-delta_y,delta_d);
  Eigen::Vector3f p_tr_=(R*p_tr)+p;
  Eigen::Vector3f p_tl_=(R*p_tl)+p;
  Eigen::Vector3f p_br_=(R*p_br)+p;
  Eigen::Vector3f p_bl_=(R*p_bl)+p;
  // Eigen::Vector3f p_tl(-delta_x,delta_y,delta_d);
  // Eigen::Vector3f p_br(delta_x,-delta_y,delta_d);
  // Eigen::Vector3f p_bl(-delta_x,-delta_y,delta_d);
  pxl pixel_p;
  pxl pixel_p_tr;
  pxl pixel_p_tl;
  pxl pixel_p_br;
  pxl pixel_p_bl;

  plotPt( p , color, pixel_p);
  plotPt( p_tr_ , color, pixel_p_tr );
  plotPt( p_tl_ , color, pixel_p_tl );
  plotPt( p_br_ , color, pixel_p_br );
  plotPt( p_bl_ , color, pixel_p_bl );

  plotLine(pixel_p,pixel_p_tr, color);
  plotLine(pixel_p,pixel_p_tl, color);
  plotLine(pixel_p,pixel_p_br, color);
  plotLine(pixel_p,pixel_p_bl, color);
  plotLine(pixel_p_tr,pixel_p_tl, color);
  plotLine(pixel_p_tl,pixel_p_bl, color);
  plotLine(pixel_p_bl,pixel_p_br, color);
  plotLine(pixel_p_br,pixel_p_tr, color);

  return true;
}
