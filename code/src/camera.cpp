#include "camera.h"

void Camera::printMembers() const {

  std::cout << "name: " << name_ << std::endl;
  cam_parameters_->printMembers();
  std::cout << "K: " << *K_ << std::endl;
  std::cout << "Kinv: " << *Kinv_ << std::endl;
  std::cout << "frame_world_wrt_camera LINEAR:\n" << (*frame_world_wrt_camera_).linear() << std::endl;
  std::cout << "frame_world_wrt_camera TRANSL:\n" << (*frame_world_wrt_camera_).translation() << std::endl;
  std::cout << "frame_camera_wrt_world LINEAR:\n" << (*frame_camera_wrt_world_).linear() << std::endl;
  std::cout << "frame_camera_wrt_world TRANSL:\n" << (*frame_camera_wrt_world_).translation() << std::endl;
  std::cout << "\n" << std::endl;

}

// sampling
void Camera::sampleRandomPixel(pxl& pixel_coords){
  pixel_coords.x() = rand() % cam_parameters_->resolution_x;
  pixel_coords.y() = rand() % cam_parameters_->resolution_y;
}
void Camera::sampleRandomUv(Eigen::Vector2f& uv){
  uv.x() = ((float)rand()/RAND_MAX) * cam_parameters_->width;
  uv.y() = ((float)rand()/RAND_MAX) * cam_parameters_->height;
}


// access
void Camera::getCenterAsUV(Eigen::Vector2f& uv) const{
  uv.x() = cam_parameters_->width/2;
  uv.y() = cam_parameters_->height/2;
}

void Camera::getCentreAsPixel(pxl& pixel_coords) const{
  pixel_coords.x() = cam_parameters_->resolution_x/2;
  pixel_coords.y() = cam_parameters_->resolution_y/2;
}

float Camera::getPixelWidth(int level) const{
  assert(level>-1);
  float pixel_width= cam_parameters_->pixel_width*pow(2,level+1);
  return pixel_width;
}


// assign
void Camera::assignPose(const Eigen::Isometry3f& frame_camera_wrt_world){
  std::lock_guard<std::mutex> locker(mu_access_pose);
  *frame_camera_wrt_world_=frame_camera_wrt_world;
  *frame_world_wrt_camera_=frame_camera_wrt_world.inverse();
}


void Camera::clearImgs(){
  invdepth_map_->setAllPixels(1.0);
}

std::shared_ptr<Eigen::Matrix3f> Camera::compute_K(){

  float lens = cam_parameters_->lens;
  float width = cam_parameters_->width;
  float height = cam_parameters_->height;

  assert(lens>0);
  assert(width>0);
  assert(height>0);

  std::shared_ptr<Eigen::Matrix3f> K (new Eigen::Matrix3f);
  *K <<
      lens ,  0   , width/2,
      0    ,  lens, height/2,
      0    ,  0   ,       1 ;

  return K;
}

void Camera::pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv, int level) const {
  assert(level>=0);
  int res_x = cam_parameters_->resolution_x;
  int res_y = cam_parameters_->resolution_y;

  int resolution_x=res_x/(pow(2,level));
  int resolution_y=res_y/(pow(2,level));
  assert( pixel_coords.x()>=0 || pixel_coords.y()>=0);
  assert( pixel_coords.x()<res_x || pixel_coords.y()<res_y );

  float ratio_x = ((float)pixel_coords.x()/(float)resolution_x);
  float ratio_y = ((float)pixel_coords.y()/(float)resolution_y);
  assert( ratio_x>=0 || ratio_x<=1 || ratio_y>=0 || ratio_y<=1 );

  uv.x()=ratio_x*cam_parameters_->width;
  uv.y()=ratio_y*cam_parameters_->height;
}

void Camera::pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv) const {
  pixelCoords2uv( pixel_coords, uv, 0);
}

Eigen::Vector2f Camera::pixelCoords2uv(const pxl& pixel_coords) const {
  Eigen::Vector2f uv;
  pixelCoords2uv( pixel_coords, uv, 0);
  return uv;
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords, int level) const {

  int resolution_x=cam_parameters_->resolution_x/(pow(2,level));
  int resolution_y=cam_parameters_->resolution_y/(pow(2,level));

  float ratio_x = (uv.x()/cam_parameters_->width);
  float ratio_y = (uv.y()/cam_parameters_->height);
  // assert( ratio_x>=0 || ratio_x<=1 || ratio_y>=0 || ratio_y<=1 );

  pixel_coords.x()=(ratio_x*(float)resolution_x);
  pixel_coords.y()=(ratio_y*(float)resolution_y);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords) const {
  uv2pixelCoords(uv, pixel_coords, 0);
}



void Camera::pointAtDepthInCamFrame(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p_incamframe ) const {

  Eigen::Vector3f p_K_dot_incamframe;
  Eigen::Vector2f product = uv * depth;
  p_K_dot_incamframe.x() = product.x();
  p_K_dot_incamframe.y() = product.y();
  p_K_dot_incamframe.z() = depth;
  p_incamframe = (*Kinv_)*p_K_dot_incamframe;

}

void Camera::pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const {

  Eigen::Vector3f p_incamframe;
  pointAtDepthInCamFrame( uv, depth, p_incamframe);
  p = *frame_camera_wrt_world_*p_incamframe;

}

void Camera::pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p, Eigen::Vector3f& p_incamframe) const {

  pointAtDepthInCamFrame( uv, depth, p_incamframe);
  p = *frame_camera_wrt_world_*p_incamframe;

}


bool Camera::projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const {


  Eigen::Vector3f p_cam = *frame_world_wrt_camera_*p;

  // save value of z
  p_cam_z=p_cam.z();

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p_cam = *frame_world_wrt_camera_*p;

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectPointInCamFrame(const Eigen::Vector3f& p_incamframe, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p_proj = (*K_)*p_incamframe;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectCam(std::shared_ptr<Camera> cam_to_be_projected, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv);
  return out;

}

bool Camera::projectCam(std::shared_ptr<Camera> cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z  ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv, p_cam_z);
  return out;

}

void Camera::saveRGB(const std::string& path) const {
  cv::imwrite(path+ "/rgb_" +name_+".png", image_intensity_->image_);
}

void Camera::saveDepthMap(const std::string& path) const {
  cv::Mat ucharImg;
  invdepth_map_->image_.convertTo(ucharImg, CV_32FC1, 255.0);
  cv::imwrite(path+ "/depth_" +name_+".png", ucharImg);

}

std::shared_ptr<Image<pixelIntensity>> Camera::returnIntensityImgFromPath(const std::string& path_rgb){

  std::shared_ptr<Image<pixelIntensity>> img (new Image<pixelIntensity>(name_));
  img->image_=cv::imread(path_rgb,cv::IMREAD_GRAYSCALE);
  img->image_.convertTo(img->image_, pixelIntensity_CODE, pixelIntensity_maxval/255.0);
  return img;
}


void Camera::loadWhiteDepth(){
  invdepth_map_->initImage(cam_parameters_->resolution_y,cam_parameters_->resolution_x);
  invdepth_map_->setAllPixels(1.0); // initialize images with white color

}

void Camera::loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f){
  float resolution_x = cam_parameters_->resolution_x;
  float resolution_y = cam_parameters_->resolution_y;

  Eigen::Matrix3f R;
  R <<
    f[0], f[1], f[2],
    f[3], f[4], f[5],
    f[6], f[7], f[8];

  Eigen::Vector3f t(f[9],f[10],f[11]);
  frame_camera_wrt_world_ = std::make_shared<Eigen::Isometry3f>();
  frame_camera_wrt_world_->linear()=R;
  frame_camera_wrt_world_->translation()=t;

  frame_world_wrt_camera_ = std::make_shared<Eigen::Isometry3f>();
  *frame_world_wrt_camera_=frame_camera_wrt_world_->inverse();

}



void Camera::loadDepthMap(const std::string& path){
  invdepth_map_->image_=cv::imread(path, cv::IMREAD_ANYDEPTH);
  invdepth_map_->image_/=1.0908;
}

void Camera::showRGB(int image_scale) const {
  image_intensity_->show(image_scale);
}

void Camera::showDepthMap(int image_scale) const {
  invdepth_map_->show(image_scale);
}
