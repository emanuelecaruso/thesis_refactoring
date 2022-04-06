#pragma once
#include "defs.h"
#include "parameters.h"
#include "json.hpp"
// #include "image.h"
#include "wavelet.h"
#include <sys/stat.h>

using namespace pr;

typedef std::pair<float,float> bound;

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
    std::mutex mu_access_pose;


    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_intensity_( image_intensity ),
           frame_camera_wrt_world_(new Eigen::Isometry3f),
           frame_world_wrt_camera_(new Eigen::Isometry3f)
           { };

    Camera(const std::string& name, const CamParameters* cam_parameters,
            const Image<pixelIntensity>* image_intensity,
            Image<float>* invdepth_map ):
            name_(name),
            cam_parameters_(cam_parameters),
            K_(compute_K()),
            Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
            image_intensity_( image_intensity ),
            invdepth_map_( invdepth_map ),
            frame_camera_wrt_world_(new Eigen::Isometry3f),
            frame_world_wrt_camera_(new Eigen::Isometry3f)
            { };

     Camera(const std::string& name, const CamParameters* cam_parameters,
            const std::string& path_rgb):
            name_(name),
            cam_parameters_(cam_parameters),
            K_(compute_K()),
            Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
            image_intensity_( returnIntensityImgFromPath( path_rgb ) ),
            frame_camera_wrt_world_(new Eigen::Isometry3f),
            frame_world_wrt_camera_(new Eigen::Isometry3f)
            { };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity, Eigen::Isometry3f* frame_world_wrt_camera,
               Eigen::Isometry3f* frame_camera_wrt_world ):
           Camera(name, cam_parameters, image_intensity )
           {
             frame_world_wrt_camera_=frame_world_wrt_camera;
             frame_camera_wrt_world_=frame_camera_wrt_world;
           };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb):
    Camera( name,cam_parameters, path_rgb )
    {
      loadPoseFromJsonVal(f);
      invdepth_map_ = new Image< float >("invdepth_"+name_);
      loadWhiteDepth();
    };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth ):
    Camera( name,cam_parameters, f, path_rgb )
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
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p, Eigen::Vector3f& p_incamframe) const;
    void pointAtDepthInCamFrame(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p_incamframe) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectPointInCamFrame(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;
    bool projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z) const;

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;


    // inline Camera* clone(){
    //   Camera* new_cam = new Camera(*this);
    //   return new_cam;
    // }
  protected:
    Eigen::Matrix3f* compute_K();
    Image<pixelIntensity>* returnIntensityImgFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);



};


class Mapper; // forward declaration

class CameraForMapping;


class CandidateBase{
  public:
    const int level_;
    pxl pixel_;
    Eigen::Vector2f uv_;

    CandidateBase(const int level, pxl& pixel, Eigen::Vector2f& uv):
    level_(level), pixel_(pixel), uv_(uv){};
};

class Candidate;

class RegionWithCandidatesBase{
  public:

    int x_, y_, reg_level_;
    float grad_threshold_;
    CameraForMapping* cam_;

    RegionWithCandidatesBase(int x, int y, Params* parameters, CameraForMapping* cam):
    x_(x), y_(y), reg_level_(parameters->reg_level),
    grad_threshold_(parameters->grad_threshold), cam_(cam)
    {    };
    void showRegion(int size);

};

class RegionWithCandidates : public RegionWithCandidatesBase{
  public:

    std::vector<Candidate*>* cands_vec_;

    RegionWithCandidates(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    cands_vec_(new std::vector<Candidate*>)
    { };
    bool collectCandidates(int wavelet_levels);

};

class Candidate : public CandidateBase{
  public:

    // create candidate
    Candidate(int level, pxl& pixel, Eigen::Vector2f& uv,
              CameraForMapping* cam,
              pixelIntensity intensity,
              float grad_magnitude,
              float grad_phase,
              // float intensity_dx, float intensity_dy,
              // float grad_magnitude_dx, float grad_magnitude_dy,
              // float grad_phase_dx, float grad_phase_dy,
              std::vector<bound>* bounds, RegionWithCandidates* region_sampling ):
    CandidateBase( level, pixel, uv),
    cam_(cam),
    bounds_(bounds),
    one_min_(false),
    invdepth_(-1),
    p_incamframe_(new Eigen::Vector3f),
    p_(new Eigen::Vector3f),
    invdepth_var_(-1),
    region_sampling_(region_sampling),
    regions_coarse_(new std::vector<RegionWithCandidates*>),
    intensity_(intensity),
    grad_magnitude_(grad_magnitude),
    grad_phase_(grad_phase)
    {};

    // candidate coarse
    Candidate(int level, pxl& pixel, Eigen::Vector2f& uv,
              CameraForMapping* cam,
              pixelIntensity intensity,
              float grad_magnitude,
              float grad_phase,
              // float intensity_dx, float intensity_dy,
              // float grad_magnitude_dx, float grad_magnitude_dy,
              // float grad_phase_dx, float grad_phase_dy,
              float invdepth, float invdepth_var, Eigen::Vector3f* p,
              Eigen::Vector3f* p_incamframe ):
    CandidateBase( level, pixel, uv),
    cam_(cam),
    bounds_(nullptr),
    one_min_(true),
    invdepth_(invdepth),
    p_incamframe_(p_incamframe),
    p_(p),
    invdepth_var_(invdepth_var),
    region_sampling_(nullptr),
    regions_coarse_(nullptr),
    intensity_(intensity),
    grad_magnitude_(grad_magnitude),
    grad_phase_(grad_phase)
    { }

    ~Candidate(){
      delete p_incamframe_;
      delete p_;
      delete regions_coarse_;
    }

    CameraForMapping* cam_;
    std::vector<bound>* bounds_;
    bool one_min_;
    float invdepth_;
    Eigen::Vector3f* p_incamframe_;
    Eigen::Vector3f* p_;
    float invdepth_var_;
    RegionWithCandidates* region_sampling_;
    std::vector<RegionWithCandidates*>* regions_coarse_;
    const pixelIntensity intensity_;
    const float grad_magnitude_;
    const float grad_phase_;

    inline float getInvdepthStandardDeviation() const{
      float d2 = bounds_->at(0).second;
      float d1 = bounds_->at(0).first;
      float sd = ((1.0/d1)-(1.0/d2))/2;
      assert(sd!=0);
      return sd;
    }

    void marginalize() const;

    void setInvdepthGroundtruth();
};



class CandidateProjected : public CandidateBase{
  public:
    const float invdepth_;
    Candidate* cand_;
    CameraForMapping* cam_;

    CandidateProjected(Candidate* cand, pxl& pixel, Eigen::Vector2f& uv, float invdepth, CameraForMapping* cam):
    CandidateBase(cand->level_, pixel, uv),
    invdepth_(invdepth),
    cand_(cand),
    cam_(cam)
    {};

};

class RegionWithActivePoints;

class ActivePoint : public CandidateBase{
  public:

    // candidate activation
    ActivePoint(Candidate* cand):
    CandidateBase( cand->level_, cand->pixel_, cand->uv_),
    cam_(cand->cam_),
    regions_coarse_(new std::vector<RegionWithActivePoints*>),

    // current guess
    invdepth_(cand->invdepth_),
    p_incamframe_( new Eigen::Vector3f ),
    p_(new Eigen::Vector3f),
    invdepth_var_(cand->invdepth_var_),
    // tangent space point
    invdepth_0_(cand->invdepth_),
    p_incamframe_0_(new Eigen::Vector3f),
    p_0_(new Eigen::Vector3f),
    // active point features
    intensity_(cand->intensity_),
    grad_magnitude_(cand->grad_magnitude_),
    grad_phase_(cand->grad_phase_),
    // delta update x
    delta_update_x_(0),

    not_seen_in_last_keyframe_(false),
    state_point_block_idx_(-1),
    state_point_block_marg_idx_(-1),
    has_occlusion_(false),
    to_marginalize_(false)
    {
      *p_incamframe_=*cand->p_incamframe_;
      *p_=*cand->p_;

      *p_incamframe_0_=*cand->p_incamframe_;
      *p_0_=*cand->p_;
    }

    // active point from corner
    ActivePoint(pxl& pixel, Eigen::Vector2f& uv,
                CameraForMapping* cam,
                pixelIntensity intensity,
                float grad_magnitude,
                float grad_phase,
                float invdepth, float invdepth_var):
    CandidateBase( 0, pixel, uv),
    cam_(cam),
    regions_coarse_(new std::vector<RegionWithActivePoints*>),

    // current guess
    invdepth_(invdepth),
    p_incamframe_( new Eigen::Vector3f ),
    p_(new Eigen::Vector3f),
    invdepth_var_(invdepth_var),
    // tangent space point
    invdepth_0_(invdepth),
    p_incamframe_0_(new Eigen::Vector3f),
    p_0_(new Eigen::Vector3f),
    // active point features
    intensity_(intensity_),
    grad_magnitude_(grad_magnitude_),
    grad_phase_(grad_phase_),
    // delta update x
    delta_update_x_(0),

    not_seen_in_last_keyframe_(false),
    state_point_block_idx_(-1),
    state_point_block_marg_idx_(-1),
    has_occlusion_(false),
    to_marginalize_(false)
    {
      initForActivationFromCorner();
    }
    void initForActivationFromCorner();

    // coarse active point
    ActivePoint(int level, pxl& pixel, Eigen::Vector2f& uv,
                CameraForMapping* cam,
                pixelIntensity intensity,
                float grad_magnitude,
                float grad_phase,
                float invdepth, float invdepth_var, Eigen::Vector3f* p,
                Eigen::Vector3f* p_incamframe ):
    CandidateBase( level, pixel, uv),
    cam_(cam),
    regions_coarse_(nullptr),

    // current guess
    invdepth_(invdepth),
    p_incamframe_(p_incamframe),
    p_(p),
    invdepth_var_(invdepth_var),
    // tangent space point
    invdepth_0_(invdepth),
    p_incamframe_0_(p_incamframe),
    p_0_(p),
    // active point features
    intensity_(intensity),
    grad_magnitude_(grad_magnitude),
    grad_phase_(grad_phase),
    // delta update x
    delta_update_x_(0),

    not_seen_in_last_keyframe_(false),
    state_point_block_idx_(-1),
    state_point_block_marg_idx_(-1),
    has_occlusion_(false),
    to_marginalize_(false)
    { }

    ~ActivePoint(){
      if(p_incamframe_!=p_incamframe_0_)
        delete p_incamframe_0_;
      if(p_!=p_0_)
        delete p_0_;

      delete p_incamframe_;
      delete p_;

    }

    CameraForMapping* cam_;
    std::vector<RegionWithActivePoints*>* regions_coarse_;

    // current guess
    float invdepth_;
    Eigen::Vector3f* p_incamframe_;
    Eigen::Vector3f* p_;
    float invdepth_var_;
    // tangent space point
    float invdepth_0_;
    Eigen::Vector3f* p_incamframe_0_;
    Eigen::Vector3f* p_0_;
    // active point features
    pixelIntensity intensity_;
    float grad_magnitude_;
    float grad_phase_;
    // delta update x
    float delta_update_x_;

    bool not_seen_in_last_keyframe_;
    int state_point_block_idx_;
    int state_point_block_marg_idx_;
    bool to_marginalize_;
    bool has_occlusion_;

    void marginalize();
    void remove();

    float getInvdepthGroundtruth();

};

class RegionWithActivePoints : public RegionWithCandidatesBase{
  public:

    std::vector<ActivePoint*>* active_pts_vec_;
    bool to_update_;

    RegionWithActivePoints(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    active_pts_vec_(new std::vector<ActivePoint*>),
    to_update_(false)
    { };

};

class ActivePointProjected : public CandidateBase{
  public:
    ActivePoint* active_point_;
    float invdepth_;
    CameraForMapping* cam_;

    // activate projected cand
    ActivePointProjected(CandidateProjected* cand_proj, ActivePoint* active_point):
    CandidateBase( cand_proj->level_, cand_proj->pixel_, cand_proj->uv_),
    invdepth_(cand_proj->invdepth_),
    active_point_(active_point),
    cam_(cand_proj->cam_)
    {}

    // project active point
    ActivePointProjected(ActivePoint* active_point, pxl& pixel, Eigen::Vector2f& uv, float invdepth, CameraForMapping* cam):
    CandidateBase(active_point->level_, pixel, uv),
    invdepth_(invdepth),
    active_point_(active_point),
    cam_(cam)
    {};

};

class RegionsWithCandidatesBase{
  public:

    RegionsWithCandidatesBase(Params* parameters, CameraForMapping* cam, int level):
    cam_(cam),
    parameters_(parameters),
    num_of_regions_x(getNRegionsX(cam,level)),
    num_of_regions_y(getNRegionsY(cam,level))
    {  };
    CameraForMapping* cam_;
    const Params* parameters_;
    const int num_of_regions_x;
    const int num_of_regions_y;

    inline int xyToIdx(int x, int y){
      return (num_of_regions_x*y+x);
    }

    int getNRegionsX(CameraForMapping* cam, int level);
    int getNRegionsY(CameraForMapping* cam, int level);
};




class RegionsWithCandidates : public RegionsWithCandidatesBase{
  public:

    RegionsWithCandidates(Params* parameters, CameraForMapping* cam, int level):
    RegionsWithCandidatesBase( parameters, cam, level),
    region_vec_(new std::vector<RegionWithCandidates*>(num_of_regions_x*num_of_regions_y) )
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithCandidates(x, y, parameters, cam);
        }
      }
    };
    std::vector<RegionWithCandidates*>* region_vec_;

    ~RegionsWithCandidates(){
      for(RegionWithCandidates* reg : *region_vec_)
        delete reg;
    }

    inline void collectCandidates(int wavelet_levels){
      for (RegionWithCandidates* reg : *region_vec_ ){
        reg->collectCandidates(wavelet_levels);
        // break;
      }
    }

};

class RegionsWithActivePoints : public RegionsWithCandidatesBase{
  public:

    RegionsWithActivePoints(Params* parameters, CameraForMapping* cam, int level):
    RegionsWithCandidatesBase( parameters, cam, level),
    region_vec_(new std::vector<RegionWithActivePoints*>(num_of_regions_x*num_of_regions_y) )
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithActivePoints(x, y, parameters, cam);
        }
      }
    };
    std::vector<RegionWithActivePoints*>* region_vec_;

    ~RegionsWithActivePoints(){
      for(RegionWithActivePoints* reg : *region_vec_){
        delete reg;
      }
      delete region_vec_;
    }

};


class RegionWithProjCandidates : public RegionWithCandidatesBase{
  public:

    std::vector<CandidateProjected*>* cands_proj_vec_;

    RegionWithProjCandidates(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    cands_proj_vec_(new std::vector<CandidateProjected*>){
    };

};


class RegionWithProjActivePoints : public RegionWithCandidatesBase{
  public:

    std::vector<ActivePointProjected*>* active_pts_proj_vec_;

    RegionWithProjActivePoints(int x, int y, Params* parameters, CameraForMapping* cam):
    RegionWithCandidatesBase( x, y, parameters, cam),
    active_pts_proj_vec_(new std::vector<ActivePointProjected*>)
    { };

};

class RegionsWithProjCandidates : public RegionsWithCandidatesBase{
  public:

    RegionsWithProjCandidates(Params* parameters, CameraForMapping* cam, int level):
    RegionsWithCandidatesBase( parameters, cam, level),
    region_vec_(new std::vector<RegionWithProjCandidates*>(num_of_regions_x*num_of_regions_y) )
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithProjCandidates(x, y, parameters, cam);
        }
      }
    };
    std::vector<RegionWithProjCandidates*>* region_vec_;

    inline void pushProjCandidate(CandidateProjected* projected_cand){


      // get associated region
      int scale_offs = pow(2,parameters_->reg_level);
      // int scale_offs = 1;
      int reg_x = projected_cand->pixel_.x()/scale_offs;
      int reg_y = projected_cand->pixel_.y()/scale_offs;
      int idx = xyToIdx( reg_x, reg_y);

      // push the projected candidate inside the region (sorted by invdepth var)
      std::vector<CandidateProjected*>* cands_vec_ = region_vec_->at(idx)->cands_proj_vec_;

      auto lb_cmp = [](CandidateProjected* const & x, float d) -> bool
        { return x->cand_->getInvdepthStandardDeviation() < d; };

      auto it = std::lower_bound(cands_vec_->begin(), cands_vec_->end(), projected_cand->cand_->getInvdepthStandardDeviation(), lb_cmp);
      cands_vec_->insert ( it , projected_cand );


    }
};


class RegionsWithProjActivePoints : public RegionsWithCandidatesBase{
// class RegionsWithProjActivePoints{
  public:

    RegionsWithProjActivePoints(Params* parameters, CameraForMapping* cam, int level):
    RegionsWithCandidatesBase( parameters, cam, level),
    region_vec_(new std::vector<RegionWithProjActivePoints*>(num_of_regions_x*num_of_regions_y) ),
    active_points_proj_(new std::vector<ActivePointProjected*>)
    {
      for(int x=0; x<num_of_regions_x; x++){
        for(int y=0; y<num_of_regions_y; y++){
          int idx=xyToIdx(x,y);
          region_vec_->at(idx)=new RegionWithProjActivePoints(x, y, parameters, cam);
        }
      }
    };

    ~RegionsWithProjActivePoints(){
      for(ActivePointProjected* active_point_proj : *active_points_proj_)
        delete active_point_proj;

      for(RegionWithProjActivePoints* reg : *region_vec_)
        delete reg;

      delete region_vec_;
      delete active_points_proj_;
    }

    std::vector<RegionWithProjActivePoints*>* region_vec_;
    std::vector<ActivePointProjected*>* active_points_proj_;


    inline void pushProjActivePoint(ActivePointProjected* proj_active_pt){

      // get associated region
      int scale_offs = pow(2,parameters_->reg_level);
      // int scale_offs = 1;
      int reg_x = proj_active_pt->pixel_.x()/scale_offs;
      int reg_y = proj_active_pt->pixel_.y()/scale_offs;
      int idx = xyToIdx( reg_x, reg_y);

      // push the projected active point inside the region (sorted by invdepth var)
      std::vector<ActivePointProjected*>* active_pt_proj_vec = region_vec_->at(idx)->active_pts_proj_vec_;
      active_pt_proj_vec->push_back(proj_active_pt);

      // push the projected active point inside the vector
      active_points_proj_->push_back(proj_active_pt);

    }

    inline int getNumOfActivePointsInReg(RegionWithProjCandidates* reg){
      int idx = xyToIdx( reg->x_, reg->y_);
      int num_proj_active_points = region_vec_->at(idx)->active_pts_proj_vec_->size();
      return num_proj_active_points;
    }
};


class PoseNormError{
  public:
    float angle;
    float position_norm;

    // default constructor
    PoseNormError():
    angle(0),
    position_norm(0)
    {  }

    // assignment constructor
    PoseNormError(float angle_, float position_error_norm):
    angle(angle_),
    position_norm(position_error_norm)
    {  }

    // += operator
    PoseNormError& operator+=(const PoseNormError& rhs){
      /* addition of rhs to *this */
      assert(!std::isnan(rhs.angle));
      assert(!std::isnan(rhs.position_norm));
      this->angle += rhs.angle;
      this->position_norm += rhs.position_norm;

      return *this; // return the result by reference
    }

    inline void print(){
      std::cout << "angle error: " << angle << std::endl;
      std::cout << "position error norm: " << position_norm << std::endl;
    }

};

class CameraForMapping: public Camera{

  public:

    Camera* grountruth_camera_;
    Wvlt_dec* wavelet_dec_;
    std::vector<Candidate*>* candidates_;
    std::vector<ActivePoint*>* active_points_;
    std::vector<ActivePoint*>* marginalized_points_;
    RegionsWithCandidates* regions_sampling_;
    RegionsWithProjCandidates* regions_projected_cands_;
    RegionsWithProjActivePoints* regions_projected_active_points_;
    std::vector<RegionsWithCandidates*>* regions_coarse_cands_vec_;
    std::vector<std::vector<Candidate*>*>* candidates_coarse_;
    std::vector<RegionsWithActivePoints*>* regions_coarse_active_pts_vec_;
    std::vector<std::vector<ActivePoint*>*>* active_points_coarse_;
    Eigen::Isometry3f* frame_camera_wrt_world_0_;
    Eigen::Isometry3f* frame_world_wrt_camera_0_;
    Vector6f* delta_update_x_;
    int num_removed_points_;
    int num_marginalized_active_points_;
    bool to_be_marginalized_ba_;
    bool to_be_marginalized_;
    bool active_points_removed_;
    bool keyframe_;
    bool fixed_;
    bool pose_updated_;
    int n_candidates_;
    int state_pose_block_idx_;
    int state_pose_block_marg_idx_;

    friend class Mapper;
    friend class Dso;

    CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
           const Image<pixelIntensity>* image_intensity, Image<float>* invdepth_map,
           Params* parameters, Camera* grountruth_camera):

           grountruth_camera_(grountruth_camera),
           Camera( name, cam_parameters, image_intensity, invdepth_map),
           wavelet_dec_(new Wvlt_dec(parameters->coarsest_level+1,new Image<pixelIntensity>(image_intensity_), this)),
           candidates_(new std::vector<Candidate*>),
           active_points_(new std::vector<ActivePoint*>),
           marginalized_points_(new std::vector<ActivePoint*>),
           regions_sampling_(new RegionsWithCandidates(parameters, this, parameters->reg_level+1)),
           regions_projected_cands_(new RegionsWithProjCandidates(parameters, this, parameters->reg_level+1)),
           regions_projected_active_points_(new RegionsWithProjActivePoints(parameters, this, parameters->reg_level+1)),
           regions_coarse_cands_vec_(new std::vector<RegionsWithCandidates*>),
           candidates_coarse_(new std::vector<std::vector<Candidate*>*>),
           regions_coarse_active_pts_vec_(new std::vector<RegionsWithActivePoints*>),
           active_points_coarse_(new std::vector<std::vector<ActivePoint*>*>),
           frame_camera_wrt_world_0_(new Eigen::Isometry3f),
           frame_world_wrt_camera_0_(new Eigen::Isometry3f),
           delta_update_x_(new Vector6f),
           num_removed_points_(0),
           num_marginalized_active_points_(0),
           to_be_marginalized_ba_(false),
           to_be_marginalized_(false),
           active_points_removed_(false),
           keyframe_(false),
           fixed_(false),
           pose_updated_(true),
           n_candidates_(0),
           state_pose_block_idx_(-1),
           state_pose_block_marg_idx_(-1)
           {
             // iterate along all coarser levels
             for(int i=1; i<=parameters->coarsest_level; i++){
               // create empty regions
               RegionsWithCandidates* coarse_cands_regions = new RegionsWithCandidates(parameters,this,i);
               regions_coarse_cands_vec_->push_back(coarse_cands_regions);

               RegionsWithActivePoints* coarse_active_pts_regions = new RegionsWithActivePoints(parameters,this,i);
               regions_coarse_active_pts_vec_->push_back(coarse_active_pts_regions);

               candidates_coarse_->push_back(new std::vector<Candidate*>);
               active_points_coarse_->push_back(new std::vector<ActivePoint*>);
             }

             delta_update_x_->setZero();
           };

    // CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
    //         const Image<pixelIntensity>* image_intensity, Eigen::Isometry3f* frame_world_wrt_camera,
    //             Eigen::Isometry3f* frame_camera_wrt_world, Params* parameters):
    //         CameraForMapping( name, cam_parameters, image_intensity, parameters)
    //         {
    //           frame_world_wrt_camera_=frame_world_wrt_camera;
    //           frame_camera_wrt_world_=frame_camera_wrt_world;
    //         };

    CameraForMapping(Camera* env_cam, Params* parameters):
           CameraForMapping( env_cam->name_, env_cam->cam_parameters_, env_cam->image_intensity_,
                              env_cam->invdepth_map_, parameters, env_cam)
           {  };

    void showCandidates(float size);
    void showCoarseCandidates(int level, float size=1);
    void showCoarseActivePoints(int level, float size=1);
    void showProjCandidates(float size);
    void showActivePoints(float size);
    void clearProjectedActivePoints();
    void showProjActivePoints(float size);
    void assignPose0(const Eigen::Isometry3f& frame_camera_wrt_world);

    PoseNormError getPoseNormError();
    float getPointsNormError();

  private:
    // void collectRegions(float grad_threshold);
    void selectNewCandidates(int max_num_candidates);

};
