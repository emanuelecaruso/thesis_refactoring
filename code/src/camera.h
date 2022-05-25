#pragma once
#include <string>
#include "defs.h"
#include "parameters.h"
#include "json.hpp"
#include "image.h"

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    const Image<pixelIntensity>* image_intensity_;
    Image<float>* invdepth_map_;
    Eigen::Isometry3f* frame_camera_wrt_world_;
    Eigen::Isometry3f* frame_world_wrt_camera_;
    float exposure_time_;
    std::mutex mu_access_pose;

    // clone camera
    Camera(Camera* cam, bool copy_pose):
      name_(cam->name_),
      cam_parameters_(cam->cam_parameters_),
      K_(cam->K_),
      Kinv_(cam->Kinv_),
      image_intensity_(cam->image_intensity_),
      invdepth_map_(cam->invdepth_map_ ),
      frame_camera_wrt_world_(new Eigen::Isometry3f ),
      frame_world_wrt_camera_(new Eigen::Isometry3f ),
      exposure_time_(cam->exposure_time_)
      {
        if(copy_pose){
          *frame_camera_wrt_world_=(*(cam->frame_camera_wrt_world_));
          *frame_world_wrt_camera_=(*(cam->frame_world_wrt_camera_));
        }
      }

    Camera(const std::string& name, const CamParameters* cam_parameters, float exposure_time=1):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_intensity_( nullptr ),
           frame_camera_wrt_world_(new Eigen::Isometry3f),
           frame_world_wrt_camera_(new Eigen::Isometry3f),
           exposure_time_(exposure_time)
           { };

     Camera(const std::string& name, const CamParameters* cam_parameters,
            const std::string& path_rgb, float exposure_time=1):
            Camera( name, cam_parameters, exposure_time)
            {
              image_intensity_=returnIntensityImgFromPath( path_rgb );
            };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb, float exposure_time=1):
    Camera( name,cam_parameters, path_rgb, exposure_time )
    {
      loadPoseFromJsonVal(f);
      invdepth_map_ = new Image< float >("invdepth_"+name_);
      loadWhiteDepth();
    };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth, float exposure_time=1 ):
    Camera( name,cam_parameters, f, path_rgb, exposure_time )
    {
      loadDepthMap(path_depth);
    };


    inline Eigen::Isometry3f access_frame_camera_wrt_world(){
      std::lock_guard<std::mutex> locker(mu_access_pose);
      Eigen::Isometry3f T = (*frame_camera_wrt_world_);
      return T;
    }

    inline Eigen::Isometry3f access_frame_world_wrt_camera(){
      std::lock_guard<std::mutex> locker(mu_access_pose);
      Eigen::Isometry3f T = (*frame_world_wrt_camera_);
      return T;
    }

    colorRGB invdepthToRgb(float invdepth);

    void printMembers() const;

    // sampling
    void sampleRandomUv(Eigen::Vector2f& uv);
    void sampleRandomPixel(pxl& pixel_coords);

    // access
    void getCenterAsUV(Eigen::Vector2f& uv) const;
    void getCentreAsPixel(pxl& pixel_coords) const;
    float getPixelWidth(int level=-1) const;

    // assign
    void assignPose(const Eigen::Isometry3f& frame_camera_wrt_world);

    // functions for projections/transformations
    void pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv, int level) const;
    void pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv) const;
    Eigen::Vector2f pixelCoords2uv(const pxl& pixel_coords) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords, int level) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords) const;
    pxl uv2pixelCoords(const Eigen::Vector2f& uv, int level) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p, Eigen::Vector3f& p_incamframe) const;
    void pointAtDepthInCamFrame(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p_incamframe) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectPointInCamFrame(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;
    bool projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z) const;
    bool uvInRange(const Eigen::Vector2f& uv);

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;



  protected:
    Eigen::Matrix3f* compute_K();
    Image<pixelIntensity>* returnIntensityImgFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);



};
