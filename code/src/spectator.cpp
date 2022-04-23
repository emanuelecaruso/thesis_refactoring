#include "spectator.h"
#include "dso.h"
#include <thread>
#include <vector>
#include <mutex>

std::shared_ptr<CamParameters> Spectator::initCamParams(){
  std::shared_ptr<CamParameters> spectator_params (new CamParameters(
  dso_->parameters_->spec_resolution_x, dso_->parameters_->spec_resolution_y, dso_->parameters_->spec_width,
  dso_->parameters_->spec_lens, dso_->parameters_->spec_min_depth, dso_->parameters_->spec_max_depth));
  return spectator_params;
}

std::shared_ptr<Camera> Spectator::initCam(){
  std::shared_ptr<Image<pixelIntensity>> null_shared_ptr(nullptr);
  std::shared_ptr<Camera> spectator_cam (new Camera("Spectator", spectator_params_, null_shared_ptr));
  return spectator_cam;
}

std::shared_ptr<Image<colorRGB>> Spectator::initImage(){
  std::shared_ptr<Image<colorRGB>> spectator_image( new Image<colorRGB> ("Spectator"));
  spectator_image->initImage(dso_->parameters_->spec_resolution_y, dso_->parameters_->spec_resolution_x);
  spectator_image->setAllPixels(background_color_);
  return spectator_image;
}

// void Spectator::spectateDso(){
//   // dso_->waitForKeyframeAdded();
//   dso_->waitForNewFrame();
//   showSpectator();
//
//   while(true){
//
//     renderState();
//     showSpectator();
//
//   }
// }
//
// void Spectator::renderState(){
//
//   reinitSpectator();
//   renderPoints();
//   renderCamsAndKFs();
//
// }
//
// void Spectator::showSpectator(){
//   spectator_image_->show(1);
//   if(!(dso_->bundle_adj_->debug_optimization_))
//     waitkey(1);
//
// }
//
// void Spectator::reinitSpectator(){
//   // std::lock_guard<std::mutex> locker(mu_cout);
//   spectator_image_->setAllPixels(background_color_);
//   Eigen::Isometry3f pose = getSpectatorPose();
//   spectator_cam_->assignPose(pose);
// }
//
// void Spectator::renderPoints(){
//   // iterate through all cameras
//   int num_kfs = dso_->frame_current_+1;
//   for ( int i = 0; i<num_kfs; i++){
//     CameraForMapping* cam = dso_->camera_vector_->at(i);
//     if (cam->keyframe_){
//       // // iterate through all marginalized points
//       // int num_marg_pts = cam->marginalized_points_->size();
//       // for ( int j = num_marg_pts-1; j>=0; j--){
//       //   ActivePoint* marg_pt = cam->marginalized_points_->at(j);
//       //   plotPt(marg_pt, black);
//       // }
//       //
//       std::vector<CameraForMapping*>* v = dso_->keyframe_vector_;
//       if (std::count(v->begin(), v->end(), cam)) {
//         int num_act_pts = cam->active_points_->size();
//         for ( int j = num_act_pts-1; j>=0; j--){
//           ActivePoint* act_pt = cam->active_points_->at(j);
//           plotPt(act_pt, magenta);
//         }
//       }
//     }
//   }
//
// }
//
// void Spectator::renderCamsAndKFs(){
//   // iterate through all cameras
//   int num_kfs = dso_->frame_current_+1;
//   for ( int i = 0; i<num_kfs; i++){
//     CameraForMapping* cam = dso_->camera_vector_->at(i);
//
//     plotCam(cam->grountruth_camera_, orange);
//     if (cam->keyframe_){
//       std::vector<CameraForMapping*>* v = dso_->keyframe_vector_;
//
//       if (std::count(v->begin(), v->end(), cam)) {
//         plotCam(cam, red);
//       }
//       else{
//         plotCam(cam, green);
//       }
//     }
//     else{
//       plotCam(cam, blue);
//     }
//
//   }
// }
//
//
// Eigen::Isometry3f Spectator::getSpectatorPose(){
//   // get first active keyframe
//   CameraForMapping* first_keyframe = dso_->camera_vector_->at(dso_->keyframe_vector_->back());
//   // distance of spectator wrt first keyframe
//   float spec_distance = dso_->parameters_->spec_distance;
//   // spec wrt first keyframe
//   Eigen::Isometry3f kf1_T_spec;
//   kf1_T_spec.linear().setIdentity();
//   kf1_T_spec.translation() << 0,0,-spec_distance;
//
//   Eigen::Isometry3f pose =  ( first_keyframe->access_frame_camera_wrt_world() )*kf1_T_spec;
//
//   return pose;
//
// }
//
// bool Spectator::plotPt(ActivePoint* pt, const colorRGB& color){
//   bool success = plotPt(*(pt->p_), color);
//   return success;
// }
//
// bool Spectator::plotPt(Eigen::Vector3f& pt, const colorRGB& color){
//   pxl pixel;
//   bool success = plotPt( pt, color, pixel);
//   return success;
// }
//
// bool Spectator::plotPt(ActivePoint* pt, const colorRGB& color, pxl& pixel){
//   bool success = plotPt(*(pt->p_), color, pixel);
//   return success;
// }
//
// bool Spectator::plotPt(Eigen::Vector3f& pt, const colorRGB& color, pxl& pixel){
//   Eigen::Vector2f uv;
//   float depth;
//   bool pt_in_front = spectator_cam_->projectPoint(pt,uv,depth);
//   if(!pt_in_front)
//     return false;
//
//   spectator_cam_->uv2pixelCoords(uv, pixel);
//
//   if(spectator_image_->pixelInRange(pixel)){
//     spectator_image_->drawCircle( color, pixel,1,2);
//     return true;
//   }
//   return false;
//
// }
//
// bool Spectator::plotLine(pxl& pixel1, pxl& pixel2, const colorRGB& color ){
//
//   if(spectator_image_->pixelInRange(pixel1) || spectator_image_->pixelInRange(pixel2) ){
//     spectator_image_->drawLine( color, pixel1, pixel2 );
//     return true;
//   }
//   return false;
// }
//
// bool Spectator::plotCam(Camera* cam, const colorRGB& color){
//
//   // Eigen::Isometry3f T;
//   Eigen::Isometry3f T = cam->access_frame_camera_wrt_world();
//   Eigen::Matrix3f R = T.linear();
//   Eigen::Vector3f p = T.translation();
//   float delta_d = dso_->parameters_->rendered_cams_size;
//   float delta_y = dso_->parameters_->rendered_cams_size/2;
//   float delta_x = delta_y*spectator_params_->aspect;
//   Eigen::Vector3f p_tr(delta_x,delta_y,delta_d);
//   Eigen::Vector3f p_tl(-delta_x,delta_y,delta_d);
//   Eigen::Vector3f p_br(delta_x,-delta_y,delta_d);
//   Eigen::Vector3f p_bl(-delta_x,-delta_y,delta_d);
//   Eigen::Vector3f p_tr_=(R*p_tr)+p;
//   Eigen::Vector3f p_tl_=(R*p_tl)+p;
//   Eigen::Vector3f p_br_=(R*p_br)+p;
//   Eigen::Vector3f p_bl_=(R*p_bl)+p;
//   // Eigen::Vector3f p_tl(-delta_x,delta_y,delta_d);
//   // Eigen::Vector3f p_br(delta_x,-delta_y,delta_d);
//   // Eigen::Vector3f p_bl(-delta_x,-delta_y,delta_d);
//   pxl pixel_p;
//   pxl pixel_p_tr;
//   pxl pixel_p_tl;
//   pxl pixel_p_br;
//   pxl pixel_p_bl;
//
//   plotPt( p , color, pixel_p);
//   plotPt( p_tr_ , color, pixel_p_tr );
//   plotPt( p_tl_ , color, pixel_p_tl );
//   plotPt( p_br_ , color, pixel_p_br );
//   plotPt( p_bl_ , color, pixel_p_bl );
//
//   plotLine(pixel_p,pixel_p_tr, color);
//   plotLine(pixel_p,pixel_p_tl, color);
//   plotLine(pixel_p,pixel_p_br, color);
//   plotLine(pixel_p,pixel_p_bl, color);
//   plotLine(pixel_p_tr,pixel_p_tl, color);
//   plotLine(pixel_p_tl,pixel_p_bl, color);
//   plotLine(pixel_p_bl,pixel_p_br, color);
//   plotLine(pixel_p_br,pixel_p_tr, color);
//
//   return true;
// }
