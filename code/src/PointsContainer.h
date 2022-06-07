#pragma once
#include "camera.h"
#include "CameraForMapping.h"

class CamCouple;
class Dso;
class PtDataForBA;

class Point{
  public:
    // ********** members **********
    CameraForMapping* cam_;
    int level_;
    pxl pixel_;
    Eigen::Vector2f uv_;

    float invdepth_ ;
    float invdepth_var_ ;

    // ********** constructor **********
    // construct without invdepth
    Point(){}
    Point(CameraForMapping* cam, pxl& pixel, int level):
    cam_(cam)
    ,level_(level)
    ,pixel_(pixel)
    ,invdepth_(-1)
    ,invdepth_var_(-1)
    {
      cam->pixelCoords2uv(pixel, uv_, level);
    }
    Point(CameraForMapping* cam, pxl& pixel, int level, float invdepth, float invdepth_var):
    cam_(cam)
    ,level_(level)
    ,pixel_(pixel)
    ,invdepth_(invdepth)
    ,invdepth_var_(invdepth_var)
    {
      cam->pixelCoords2uv(pixel, uv_, level);
    }

    // ********** methods **********

};

class Candidate : public Point{
  public:

  // ********** members **********
  float depth_min_;
  float depth_max_;

  pixelIntensity c_;
  pixelIntensity magn_cd_;
  pixelIntensity phase_cd_;

  float cost_threshold_intensity_;
  float cost_threshold_gradient_;
  float cost_threshold_ba_intensity_;
  float cost_threshold_ba_gradient_;
  // CameraForMapping* cam_;
  // int level_;
  // pxl pixel_;
  // Eigen::Vector2f uv_;

  // ********** constructor **********
  // construct candidate from pixel
  Candidate(CameraForMapping* cam, pxl& pixel, int level):
  Point(cam, pixel, level)
  ,depth_min_(cam->cam_parameters_->min_depth)
  ,depth_max_(cam->cam_parameters_->max_depth)
  ,c_(cam->pyramid_->getC(level)->evalPixel(pixel))
  ,magn_cd_(cam->pyramid_->getMagn(level)->evalPixel(pixel))
  ,phase_cd_(cam->pyramid_->getPhase(level)->evalPixel(pixel))
  {}

  // ********** methods **********
  void showCandidate();
  void setInvdepthGroundtruth();
  void remove();


};


class CandidateProjected : public Point{
  public:

  // ********** members **********
  Candidate* cand_;

  // ********** constructor **********
  CandidateProjected(Candidate* cand, std::shared_ptr<CamCouple> cam_couple_ ):
  Point()
  ,cand_( cand )
  {
    init(cand,cam_couple_);
  }

  // ********** methods **********
  void init(Candidate* cand, std::shared_ptr<CamCouple> cam_couple_);


};

class ActivePoint : public Point{
public:

  // ********** members **********
  Eigen::Vector3f p_;
  Eigen::Vector3f p_incamframe_;
  pixelIntensity c_;
  pixelIntensity magn_cd_;
  int p_idx_;
  bool new_;
  float cost_threshold_ba_intensity_;
  float cost_threshold_ba_gradient_;
  std::vector<pixelIntensity> c_level_vec_;
  std::vector<pixelIntensity> magn_cd_level_vec_;

  // current guess
  // invdepth_(cand->invdepth_),
  // p_incamframe_( new Eigen::Vector3f ),
  // p_(new Eigen::Vector3f),
  // invdepth_var_(cand->invdepth_var_),

  // ********** constructor **********

  // create from activation of candidate
  ActivePoint(Candidate* cand):
  Point(cand->cam_, cand->pixel_, cand->level_ )
  ,c_(cand->c_)
  ,magn_cd_(cand->magn_cd_)
  ,p_idx_(-1)
  ,new_(true)
  ,cost_threshold_ba_intensity_(cand->cost_threshold_ba_intensity_)
  ,cost_threshold_ba_gradient_(cand->cost_threshold_ba_gradient_)
  {
    updateInvdepthVarAndP(cand->invdepth_, cand->invdepth_var_);

    c_level_vec_.resize(coarsest_level);
    magn_cd_level_vec_.resize(coarsest_level);


    for (int level=0; level<coarsest_level; level++){
      if(level==0){
        c_level_vec_[0] = cand->c_;
        magn_cd_level_vec_[0] = cand->magn_cd_;
      }
      else{
        pxl pixel_coarse;
        cam_->uv2pixelCoords( uv_, pixel_coarse, level);
        c_level_vec_[level] = cand->cam_->pyramid_->getC(level)->evalPixel(pixel_coarse);
        magn_cd_level_vec_[level] = cand->cam_->pyramid_->getMagn(level)->evalPixel(pixel_coarse);
      }
    }

  }

  // ********** methods **********
  void updateInvdepthVarAndP( float invdepth, float invdepth_var);
  void remove();

};


class MarginalizedPoint : public Point{
public:

  // ********** members **********
  Eigen::Vector3f p_;
  PtDataForBA* pt_data_for_ba_;
  // data for bundle adjustment

  // ********** constructor **********

  // create from active point
  MarginalizedPoint(ActivePoint* active_pt):
  Point(active_pt->cam_, active_pt->pixel_, active_pt->level_ ),
  p_(active_pt->p_),
  pt_data_for_ba_(initializeDataForBA())
  {}


  // ********** methods **********
  void remove();
  PtDataForBA* initializeDataForBA();

};

class MarginalizedPointProjected : public Point{
public:

  // ********** members **********
  MarginalizedPoint* marg_pt_;

  // ********** constructor **********
  // project active point
  MarginalizedPointProjected(MarginalizedPoint* marg_pt, std::shared_ptr<CamCouple> cam_couple_ ):
  Point()
  ,marg_pt_( marg_pt )
  {
    init(marg_pt,cam_couple_);
  }

  // ********** methods **********
  void init(MarginalizedPoint* marg_pt, std::shared_ptr<CamCouple> cam_couple_);


};


class ActivePointProjected : public Point{
public:

  // ********** members **********
  ActivePoint* active_pt_;

  // ********** constructor **********
  // project active point
  ActivePointProjected(ActivePoint* active_pt, std::shared_ptr<CamCouple> cam_couple_ ):
  Point()
  ,active_pt_( active_pt )
  {
    init(active_pt,cam_couple_);
  }

  // activate candidate projected
  ActivePointProjected(ActivePoint* active_pt, CandidateProjected* cand_proj ):
  Point(cand_proj->cam_, cand_proj->pixel_, cand_proj->level_, cand_proj->invdepth_, cand_proj->invdepth_var_)
  ,active_pt_(active_pt){ }



  // ********** methods **********
  void init(ActivePoint* active_pt, std::shared_ptr<CamCouple> cam_couple_);
  bool checkOutlier();


};

class PointsContainer{
  public:
    // ********** members **********
    CameraForMapping* cam_;
    std::vector<Candidate*> candidates_;
    std::vector<CandidateProjected*> candidates_projected_;
    std::vector<ActivePoint*> active_points_;
    std::vector<ActivePointProjected*> active_points_projected_;
    std::vector<MarginalizedPoint*> marginalized_points_;
    std::vector<MarginalizedPointProjected*> marginalized_points_projected_;
    int n_active_points_removed_;


    // ********** constructor **********
    PointsContainer(CameraForMapping* cam ):
    cam_(cam)
    ,n_active_points_removed_(0)
    {};


    ~PointsContainer(){
      for( Candidate* cand : candidates_ )
        delete cand;
      for( CandidateProjected* cand_proj : candidates_projected_ )
        delete cand_proj;
      for( ActivePoint* active_pt : active_points_ )
        delete active_pt;
      for( ActivePointProjected* active_pt_proj : active_points_projected_ )
        delete active_pt_proj;
      for( MarginalizedPoint* marg_pt : marginalized_points_ )
        delete marg_pt;
      for( MarginalizedPointProjected* marg_pt_proj : marginalized_points_projected_ )
        delete marg_pt_proj;

    }


    // ********** methods **********
    void drawPoint(Point* point, Image<colorRGB>* show_img, bool circle=true);
    std::vector<ActivePoint*>& getActivePoints();
    std::vector<ActivePoint*>& getActivePoints(int level);
    void showCandidates();
    void showProjectedCandidates();
    void showProjectedCandidates( const std::string& name );
    void showActivePoints();
    void showProjectedActivePoints( int wtk=0);
    void showProjectedActivePoints( const std::string& name, int wtk=0 );
    void clearProjections();

  protected:
};
