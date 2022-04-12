#pragma once
#include "camera.h"
#include "CameraForMapping.h"

class CamCouple;

class Point{
  public:
    // ********** members **********
    std::shared_ptr<CameraForMapping> cam_;
    int level_;
    pxl pixel_;
    Eigen::Vector2f uv_;

    float invdepth_ ;
    float invdepth_var_ ;

    // ********** constructor **********
    // construct without invdepth
    Point(){}
    Point(std::shared_ptr<CameraForMapping> cam, pxl& pixel, int level):
    cam_(cam)
    ,level_(level)
    ,pixel_(pixel)
    ,invdepth_(-1)
    ,invdepth_var_(-1)
    {
      cam->pixelCoords2uv(pixel, uv_, level);
    }

};

class Candidate : public Point{
  public:

  // ********** members **********
  float depth_min_;
  float depth_max_;

  pixelIntensity c_;
  pixelIntensity magn_cd_;
  pixelIntensity phase_cd_;
  // std::shared_ptr<CameraForMapping> cam_;
  // int level_;
  // pxl pixel_;
  // Eigen::Vector2f uv_;

  // ********** constructor **********
  // construct candidate from pixel
  Candidate(std::shared_ptr<CameraForMapping> cam, pxl& pixel, int level):
  Point(cam, pixel, level)
  ,depth_min_(cam->cam_parameters_->min_depth)
  ,depth_max_(cam->cam_parameters_->max_depth)
  ,c_(cam->pyramid_->getC(level)->evalPixel(pixel))
  ,magn_cd_(cam->pyramid_->getMagn(level)->evalPixel(pixel))
  ,phase_cd_(cam->pyramid_->getPhase(level)->evalPixel(pixel))
  {}

  // ********** methods **********
  void showCandidate();

};


class CandidateProjected : public Point{
  public:

  // ********** members **********
  std::shared_ptr<Candidate> cand_;

  // ********** constructor **********
  CandidateProjected(std::shared_ptr<Candidate> cand, std::shared_ptr<CamCouple> cam_couple_ ):
  Point()
  ,cand_( cand )
  {
    init(cand,cam_couple_);
  }

  // ********** methods **********
  void init(std::shared_ptr<Candidate> cand, std::shared_ptr<CamCouple> cam_couple_);


};

class ActivePoint : public Point{
public:

  // ********** members **********
  Eigen::Vector3f p_ ;
  Eigen::Vector3f p_incamframe_ ;
  // std::shared_ptr<CameraForMapping> cam_;
  // int level_;
  // pxl pixel_;
  // Eigen::Vector2f uv_;
  //
  // float invdepth_;
  // Eigen::Vector3f p_incamframe_;
  // Eigen::Vector3f p_;
  // float invdepth_var_;

  // current guess
  // invdepth_(cand->invdepth_),
  // p_incamframe_( new Eigen::Vector3f ),
  // p_(new Eigen::Vector3f),
  // invdepth_var_(cand->invdepth_var_),

  // ********** constructor **********
  ActivePoint(std::shared_ptr<CameraForMapping> cam, pxl& pixel, int level):
  Point(cam, pixel, level){}

  // ********** methods **********

};


class ActivePointProjected : public Point{
public:

  // ********** members **********
  std::shared_ptr<ActivePoint> active_pt_;

  // ********** constructor **********
  ActivePointProjected(std::shared_ptr<ActivePoint> active_pt, std::shared_ptr<CamCouple> cam_couple_ ):
  Point()
  ,active_pt_( active_pt )
  {
    init(active_pt,cam_couple_);
  }

  // ********** methods **********
  void init(std::shared_ptr<ActivePoint> active_pt, std::shared_ptr<CamCouple> cam_couple_);



};

class PointsContainer{
  public:
    // ********** members **********
    std::shared_ptr<Params> parameters_;
    std::shared_ptr<CameraForMapping> cam_;
    std::shared_ptr<std::vector<std::shared_ptr<Candidate>>> candidates_;
    std::shared_ptr<std::vector<std::shared_ptr<CandidateProjected>>> candidates_projected_;
    std::shared_ptr<std::vector<std::shared_ptr<ActivePoint>>> active_points_;
    std::shared_ptr<std::vector<std::shared_ptr<ActivePointProjected>>> active_points_projected_;

    // ********** constructor **********
    PointsContainer(CameraForMapping* cam, std::shared_ptr<Params> parameters):
    parameters_(parameters)
    ,cam_(cam)
    ,candidates_(new std::vector<std::shared_ptr<Candidate>>)
    ,candidates_projected_(new std::vector<std::shared_ptr<CandidateProjected>>)
    ,active_points_(new std::vector<std::shared_ptr<ActivePoint>>)
    ,active_points_projected_(new std::vector<std::shared_ptr<ActivePointProjected>>)
    {};


    // ********** methods **********
    void drawPoint(std::shared_ptr<Point> point, std::shared_ptr<Image<colorRGB>> show_img);
    void showCandidates();
    void showProjectedCandidates();
    void showActivePoints();
    void showProjectedActivePoints();

  protected:
};
