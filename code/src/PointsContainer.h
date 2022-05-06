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
  CandidateProjected(Candidate* cand, CamCouple* cam_couple_ ):
  Point()
  ,cand_( cand )
  {
    init(cand,cam_couple_);
  }

  // ********** methods **********
  void init(Candidate* cand, CamCouple* cam_couple_);


};

class ActivePoint : public Point{
public:

  // ********** members **********
  Eigen::Vector3f p_;
  Eigen::Vector3f p_incamframe_;
  pixelIntensity c_;
  pixelIntensity magn_cd_;
  int p_idx_;
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
  {
    updateInvdepthVarAndP(cand->invdepth_, cand->invdepth_var_);
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
  MarginalizedPointProjected(MarginalizedPoint* marg_pt, CamCouple* cam_couple_ ):
  Point()
  ,marg_pt_( marg_pt )
  {
    init(marg_pt,cam_couple_);
  }

  // ********** methods **********
  void init(MarginalizedPoint* marg_pt, CamCouple* cam_couple_);


};


class ActivePointProjected : public Point{
public:

  // ********** members **********
  ActivePoint* active_pt_;

  // ********** constructor **********
  // project active point
  ActivePointProjected(ActivePoint* active_pt, CamCouple* cam_couple_ ):
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
  void init(ActivePoint* active_pt, CamCouple* cam_couple_);



};

class PointsContainer{
  public:
    // ********** members **********
    Params* parameters_;
    CameraForMapping* cam_;
    std::vector<Candidate*> candidates_;
    std::vector<CandidateProjected*> candidates_projected_;
    std::vector<ActivePoint*> active_points_;
    std::vector<ActivePointProjected*> active_points_projected_;
    std::vector<MarginalizedPoint*> marginalized_points_;
    std::vector<MarginalizedPointProjected*> marginalized_points_projected_;
    int n_active_points_removed_;


    // ********** constructor **********
    PointsContainer(CameraForMapping* cam, Params* parameters):
    parameters_(parameters)
    ,cam_(cam)
    ,n_active_points_removed_(0)
    {};


    // ********** methods **********
    void drawPoint(Point* point, Image<colorRGB>* show_img, bool circle=true);
    std::vector<ActivePoint*>& getActivePoints();
    std::vector<ActivePoint*>& getActivePoints(int level);
    void showCandidates();
    void showProjectedCandidates();
    void showActivePoints();
    void showProjectedActivePoints();
    void showProjectedActivePoints( const std::string& name );

  protected:
};
