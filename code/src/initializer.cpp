#include "dso.h"
#include "initializer.h"
#include "CameraForMapping.h"
// #include "mapper.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <opencv2/core/eigen.hpp>

// const Image<float>* Initializer::getReferenceImage(){
//   return dso_->camera_vector_->at(ref_frame_idx_)->image_intensity_;
// }
//
// const Image<float>* Initializer::getCurrentImage(){
//   return dso_->camera_vector_->at(dso_->frame_current_)->image_intensity_;
// }
//
// const Image<float>* Initializer::getPrevImage(){
//   return dso_->camera_vector_->at(dso_->frame_current_-1)->image_intensity_;
// }
//
void Initializer::compute_cv_K(){
  // Intrinsic parameters used in opencv are expressed differently:
  // uv coordinates are expressed in pixels units

  float pixel_meter_ratio = dso_->cam_parameters_->pixel_meter_ratio;
  // express focal length (lens) and principal point in pixels
  float f_in_pixels = dso_->cam_parameters_->lens*pixel_meter_ratio;
  float pp_x = dso_->cam_parameters_->resolution_x/2;
  float pp_y = dso_->cam_parameters_->resolution_y/2;

  // compute camera matrix K for opencv
  Eigen::Matrix3f K;
  K <<  f_in_pixels ,  0          ,  pp_x,
        0           ,  f_in_pixels,  pp_y,
        0           ,  0          ,  1 ;

  eigen2cv(K, cv_K);
}

void Initializer::extractCorners(){

  corners_vec_->clear();
  errors_vec_->clear();
  status_vec_->clear();
  inliers_vec_->clear();
  compute_cv_K();

  ref_frame_=dso_->frame_current_;
  ref_frame_idx_=dso_->frame_current_idx_;
  corners_vec_->push_back(new std::vector<cv::Point2f>);
  errors_vec_->push_back(new std::vector<float>);
  status_vec_->push_back(new std::vector<uchar>);
  inliers_vec_->push_back(new std::vector<uchar>);
  cv::goodFeaturesToTrack(ref_frame_->image_intensity_->image_,*(corners_vec_->at(0)),dso_->parameters_->n_corners,dso_->parameters_->quality_level,dso_->parameters_->min_distance);


}

// void Initializer::showCornersRef(){
//   const Image<float>* img_r = getReferenceImage();
//   Image<colorRGB>* show_image= img_r->returnColoredImgFromIntensityImg("corners: "+std::to_string(corners_vec_->at(0)->size()));
//
//   for (auto corner : *(corners_vec_->at(0))){
//     // corner.x=(corner.x/dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->width)*dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->resolution_x;
//     // corner.y=(corner.y/dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->height)*dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->resolution_y;
//     show_image->drawCircle(red, corner);
//   }
//   show_image->show(2);
//   waitkey(0);
//   delete show_image;
// }

void Initializer::trackCornersLK(){
  std::shared_ptr<Image<float>> img_prev = ref_frame_->image_intensity_;
  std::shared_ptr<Image<float>> img_next = dso_->frame_current_->image_intensity_;
  cv::Mat_<uchar> img_prev_uchar;
  cv::Mat_<uchar> img_next_uchar;

  int n = dso_->frame_current_idx_-ref_frame_idx_;

  // calculate optical flow
  cv::Size size_win = cv::Size(dso_->parameters_->size_window,dso_->parameters_->size_window);

  corners_vec_->push_back(new std::vector<cv::Point2f>);
  errors_vec_->push_back(new std::vector<float>);
  status_vec_->push_back(new std::vector<uchar>);
  inliers_vec_->push_back(new std::vector<uchar>);

  img_prev->image_.convertTo(img_prev_uchar, CV_8UC1, 255);
  img_next->image_.convertTo(img_next_uchar, CV_8UC1, 255);


  // TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
  calcOpticalFlowPyrLK(img_prev_uchar, img_next_uchar, *(corners_vec_->at(n-1)), *(corners_vec_->at(n)), *(status_vec_->at(n)), *(errors_vec_->at(n)), size_win);

  // filter corners
  for (int i=status_vec_->at(n)->size()-1; i>=0; i--){
    // if(errors_vec_->at(n)->at(i)<10 || !(status_vec_->at(n)->at(i)) ){
    if(errors_vec_->at(n)->at(i)>dso_->parameters_->err_threshold ){
      for (int j=0; j<=n; j++){
        corners_vec_->at(j)->erase (corners_vec_->at(j)->begin()+i);
        if(j>0){
          status_vec_->at(j)->erase (status_vec_->at(j)->begin()+i);
          errors_vec_->at(j)->erase (errors_vec_->at(j)->begin()+i);
        }
      }
    }

  }




  // for (int i=0; i<status_vec_->at(n)->size(); i++){
  //   status_vec_->at(n)->at(i)=errors_vec_->at(n)->at(i)<10;
  // }
}

bool Initializer::findPose(){
  // estimate homography
  // cv::Mat H = findHomography();

  // estimate essential matrix
  cv::Mat E = findEssentialMatrix();
  // cv::Mat F = findFundamentalMatrix();
  // cv::Mat E = fundamental2Essential(F);

  // eval models

  // if the model is good enough
  if(true){
    // find pose
    Eigen::Isometry3f T = essential2pose( E );
    // Eigen::Isometry3f T = homography2pose( H );

    // assign pose
    std::shared_ptr<CameraForMapping> cam = dso_->frame_current_;

    cam->assignPose(T);
    // cam->assignPose0(T);


    // dso_->camera_vector_->at(dso_->frame_current_)->assignPose(*(dso_->environment_->camera_vector_->at(dso_->frame_current_)->frame_camera_wrt_world_));  //gt
    return true;
  }
  // otherwise return false
  return false;

}
//
// void Initializer::corners2activePoints(){
//
//   // CameraForMapping* cam_r =dso_->camera_vector_->at(ref_frame_idx_);
//   // CameraForMapping* cam_m =dso_->camera_vector_->at(ref_frame_idx_+1);
//   // std::cout << "cam r m " << cam_r->name_ << " " << cam_m->name_ << std::endl;
//   //
//   // // cam couple
//   // CamCouple cam_couple(cam_r,cam_m);
//   //
//   // // iterate through all corners
//   // for (int i=0; i<corners_vec_->at(1)->size(); i++) {
//   //
//   //   if(inliers_vec_->at(1)->at(i)){
//   //
//   //     cv::Point2f& corner_r = corners_vec_->at(0)->at(i);
//   //     cv::Point2f& corner_m = corners_vec_->at(1)->at(i);
//   //
//   //     pxl pixel_r(corner_r.x/2,corner_r.y/2);
//   //     pxl pixel_m(corner_m.x/2,corner_m.y/2);
//   //     Eigen::Vector2f uv_r;
//   //     Eigen::Vector2f uv_m;
//   //     cam_r->pixelCoords2uv(pixel_r,uv_r,0);
//   //     cam_m->pixelCoords2uv(pixel_m,uv_m,0);
//   //
//   //     float d1;
//   //
//   //     cam_couple.getD1(uv_r.x(),uv_r.y(),d1,uv_m.x(),uv_m.y());
//   //
//   //     pixelIntensity c = cam_r->wavelet_dec_->getWavLevel(0)->c->evalPixel( pixel_r.y(), pixel_r.x() );
//   //     pixelIntensity magn_cd = cam_r->wavelet_dec_->getWavLevel(0)->magn_cd->evalPixel( pixel_r.y(), pixel_r.x() );
//   //     pixelIntensity phase_cd = cam_r->wavelet_dec_->getWavLevel(0)->phase_cd->evalPixel( pixel_r.y(), pixel_r.x() );
//   //
//   //     // create active point from corner
//   //     ActivePoint* active_point = new ActivePoint(pixel_r, uv_r, cam_r, 0, 0, 0, 1.0/d1, 0.001);
//   //     // push active point
//   //     active_point->cam_->active_points_->push_back(active_point);
//   //
//   //     dso_->bundle_adj_->num_active_points_++;
//   //
//   //   }
//   //   dso_->bundle_adj_->collectCoarseActivePoints();
//   //   }
//
// }
//
Eigen::Isometry3f Initializer::computeRelativePoseGt(){
  Eigen::Isometry3f w_T_m = *(dso_->frame_current_->grountruth_camera_->frame_camera_wrt_world_);
  Eigen::Isometry3f r_T_w = *(ref_frame_->frame_world_wrt_camera_);
  Eigen::Isometry3f r_T_m = r_T_w*w_T_m;

  return r_T_m;
}


// cv::Mat Initializer::fundamental2Essential(cv::Mat& F_){
//
//   Eigen::Matrix3f K = *(dso_->camera_vector_->at(dso_->frame_current_)->K_);
//   Eigen::Matrix3f Kt = K.transpose();
//   Eigen::Matrix3f F;
//   cv2eigen(  F_, F );
//
//   Eigen::Matrix3f E = Kt*F*K;
//
//   cv::Mat E_;
//   eigen2cv(  E, E_ );
//
//   return E_;
//
// }
//
// Eigen::Isometry3f Initializer::homography2pose(cv::Mat& H){
//   // get grountruth of the pose to predict
//   Eigen::Isometry3f T_gt = computeRelativePoseGt();
//    // get groundtruth of scale
//   float t_magnitude = T_gt.translation().norm();
//
//   Eigen::Matrix3f K_ = *(dso_->camera_vector_->at(dso_->frame_current_)->K_);
//   cv::Mat K;
//   eigen2cv(K_, K);
//   std::vector<cv::Mat> Rs, Ts;
//
//   cv::decomposeHomographyMat(H, K, Rs, Ts, cv::noArray());
//
//   // std::cout << "\nCOMPARISON, frame " << dso_->frame_current_ << std::endl;
//   // std::cout << "gt: "<< T_gt.translation() << std::endl;
//   // std::cout << "pred: " << Ts[0] << std::endl;
//
//
//   Eigen::Isometry3f T;
//   // T.linear()=R;
//   return T;
// }
//
//
Eigen::Isometry3f Initializer::essential2pose(cv::Mat& E){

  // get grountruth of the pose to predict
  Eigen::Isometry3f T_gt = computeRelativePoseGt();
   // get groundtruth of scale
  float t_magnitude = T_gt.translation().norm();

  // compute relative pose with opencv
  cv::Mat R, t;
  cv::recoverPose	(	E, *(corners_vec_->at(0)), *(corners_vec_->back()),
                    cv_K, R, t, *(inliers_vec_->back()) );

  int i =0;
  for (uchar inlier : *(inliers_vec_->back())) {
    if (inlier){
      i++;
    }
  }
  // std::cout << "Inliers: " << i << " out of " << inliers_vec_->back()->size() << std::endl;

  Eigen::Isometry3f r_T_m;
  Eigen::Matrix3f R_;
  Eigen::Vector3f t_;
  cv2eigen(R,R_);
  cv2eigen(t,t_);
  r_T_m.linear() = R_;
  r_T_m.translation()=t_;
  // solution given by opencv: world wrt the camera -> need inversion
  r_T_m.translation()*=t_magnitude;
  r_T_m=r_T_m.inverse();


  // std::cout << "ref: " << ref_frame_idx_ << ", last: " << corners_vec_->size()-1 << std::endl;
  // std::cout << "gt: "<< T_gt.translation() << std::endl;
  // std::cout << "pred: " << r_T_m.translation() << std::endl;
  // std::cout << "gt normalized: "<< T_gt.translation().normalized() << std::endl;
  // std::cout << "pred normalized cv: " << r_T_m.translation().normalized() << std::endl;

  // relative pose to camera pose
  Eigen::Isometry3f w_T_r = *(ref_frame_->frame_camera_wrt_world_);
  Eigen::Isometry3f frame_camera_wrt_world = w_T_r*r_T_m;

  // return the camera pose
  return frame_camera_wrt_world;
}



cv::Mat Initializer::findEssentialMatrix(){
  int method = cv::RANSAC;
  double prob = dso_->parameters_->confidence;
  double threshold = dso_->parameters_->ransacReprojThreshold;

  // Eigen::Matrix3f K_ = *(dso_->camera_vector_->at(dso_->frame_current_)->K_);
  // cv::Mat K;
  // eigen2cv(K_, K);



  cv::Mat E = cv::findEssentialMat ( *(corners_vec_->at(ref_frame_idx_)), *(corners_vec_->back()),
                                      cv_K, method, prob, threshold, *(inliers_vec_->back() ) );

  int i =0;
  for (uchar inlier : *(inliers_vec_->back())) {
    if (inlier){
      i++;
    }
  }
  // std::cout << "Inliers: " << i << " out of " << inliers_vec_->back()->size() << std::endl;

  return E;
}
//
// cv::Mat Initializer::findFundamentalMatrix(){
//   int method = cv::FM_RANSAC;
//   double ransacReprojThreshold = parameters_->ransacReprojThreshold;
//   double 	confidence = parameters_->confidence;
//
//   cv::Mat F = cv::findFundamentalMat	(	*(corners_vec_->at(0)), *(corners_vec_->back()),
//                                         method, ransacReprojThreshold, confidence, *(inliers_vec_->back()) );
//
//   int i =0;
//   for (uchar inlier : *(inliers_vec_->back())) {
//     if (inlier){
//       i++;
//     }
//   }
//   // std::cout << "Inliers: " << i << " out of " << inliers_vec_->back()->size() << std::endl;
//   // cv::Mat F = cv::findFundamentalMat	(	*(corners_vec_->at(0)), *(corners_vec_->back()) );
//
//   return F;
// }
//
// cv::Mat Initializer::findHomography(){
//   int method = cv::RANSAC;
//   double ransacReprojThreshold = parameters_->ransacReprojThreshold;
//   const int maxIters = 2000;
//   double 	confidence = parameters_->confidence;
//
//
//   cv::Mat H = cv::findHomography	( *(corners_vec_->at(0)), *(corners_vec_->back()),
//                           method, ransacReprojThreshold, cv::noArray(), maxIters, confidence );
//
//
//   return H;
// }
//
void Initializer::initializeColors(){
  for (int i=0; i<corners_vec_->at(0)->size(); i++){
    float r = ((float)rand()/RAND_MAX);
    float g = ((float)rand()/RAND_MAX);
    float b = ((float)rand()/RAND_MAX);
    colorRGB color = colorRGB {b,g,r};
    colors.push_back(color);
  }
}

void Initializer::showCornersTrackCurr(int i){

    // for(int i=0; i<corners_vec_->size(); i++){

  if (i==0)
    initializeColors();

  std::shared_ptr<CameraForMapping> cam_r =ref_frame_;
  std::shared_ptr<CameraForMapping> cam_m =dso_->frame_current_;
  std::shared_ptr<Image<colorRGB>> show_image(cam_m->image_intensity_->returnColoredImgFromIntensityImg("corners tracking"));

  for (int j=0; j<corners_vec_->at(i)->size(); j++){
    cv::Point2f corner = corners_vec_->at(i)->at(j);

    show_image->drawCircle(colors[j], corner);
  }
  // if (i>0){
  //   CamCouple* cam_couple = new CamCouple(cam_r,cam_m);
  //   for (int j=0; j<corners_vec_->at(i)->size(); j++){
  //     float u= (corners_vec_->at(ref_frame_idx_)->at(j).x/dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->resolution_x)*dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->width;
  //     float v= (corners_vec_->at(ref_frame_idx_)->at(j).y/dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->resolution_y)*dso_->camera_vector_->at(ref_frame_idx_)->cam_parameters_->height;
  //
  //     EpipolarLine* ep_line = cam_couple->getEpSegmentDefaultBounds(u,v);
  //     if ( inliers_vec_->at(i)->at(j) )
  //     ep_line->drawEpipolar(show_image, colors[j] );
  //   }
  // }
  show_image->show(2);
  waitkey(0);
  // delete show_image;

}

void Initializer::showCornersTrackCurr(){
  showCornersTrackCurr(corners_vec_->size()-1);
}

// void Initializer::showCornersTrackSequence(){
//
//   while(true){
//     for(int i=0; i<corners_vec_->size(); i++){
//       showCornersTrackCurr(i);
//     }
//
//   }
// }
