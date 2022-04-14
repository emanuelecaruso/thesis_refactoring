#pragma once
#include "camera.h"
#include "PointsContainer.h"


// vector for each level of coarse active points
class ActptsCoarseVec{
  public:
    // ********** members **********
    std::vector<std::vector<std::shared_ptr<ActivePoint>>> vec_;

    // ********** constructor **********
    ActptsCoarseVec(int levels){
      vec_.resize(levels);
    }

    // ********** methods **********
    inline void push(std::shared_ptr<ActivePoint> active_pt){
      vec_[active_pt->level_].push_back(active_pt);
    }
};

// coarse region
class CoarseRegion : public Point{
  public:
    // ********** members **********
    std::shared_ptr<CoarseRegion> parent_;
    std::vector<std::shared_ptr<CoarseRegion>> subregions_;
    std::vector<std::shared_ptr<ActivePoint>> active_pts_;


    // ********** constructor **********
    CoarseRegion(std::shared_ptr<CameraForMapping> cam, pxl& pixel, int level):
    Point(cam, pixel, level)
    ,parent_(nullptr)
    {}

    // ********** methods **********
    void updateFromActivePts();
    void updateFromSubregions();
    std::shared_ptr<ActivePoint> generateCoarseActivePoint();

};

// matrix of coarse regions
class CoarseRegionMat{
  public:
    // ********** members **********
    std::shared_ptr<CoarseRegion>** mat_;
    unsigned int nrows_;
    unsigned int ncols_;

    // ********** constructor **********
    CoarseRegionMat(int nrows, int ncols){
      mat_=new std::shared_ptr<CoarseRegion>*[nrows];
      for( int i = 0; i < nrows; i++ ) {
        mat_[i] = new std::shared_ptr<CoarseRegion>[ncols];
        for( int j = 0; j < ncols; j++ ) {
          std::shared_ptr<CoarseRegion> ptr(nullptr);
          mat_[i][j] = ptr;
        }
      }
    }

    // ********** methods **********

};

// vector for each level of coarse regions matrices
class CoarseRegionMatVec{
  public:
    // ********** members **********
    std::vector<std::shared_ptr<CoarseRegionMat>> coarseregionsmat_vec_;


    // ********** constructor **********
    CoarseRegionMatVec(std::shared_ptr<PointsContainer> points_container, int levels){
      init(points_container, levels);
    }

    // ********** methods **********
    void init(std::shared_ptr<PointsContainer> points_container, int levels);

};

class CoarseRegions{
  public:
    // ********** members **********
    std::shared_ptr<PointsContainer> points_container_;
    int levels_;
    CoarseRegionMatVec coarseregionsmat_vec_;
    ActptsCoarseVec actptscoarse_vec_;
    std::vector<std::vector<std::shared_ptr<CoarseRegion>>> coarseregions_vec_;


    // ********** constructor **********
    CoarseRegions(std::shared_ptr<PointsContainer> points_container, int levels):
    points_container_(points_container)
    ,levels_(levels)
    ,coarseregionsmat_vec_(points_container, levels)
    ,actptscoarse_vec_(levels)
    { }

    // ********** methods **********
    void generateCoarseActivePoints();
    void addActivePoint(std::shared_ptr<ActivePoint> active_pt);
    void removeActivePoint(std::shared_ptr<ActivePoint> active_pt);
    void showCoarseLevel(int level);

  protected:
    void getLevelRowCol(std::shared_ptr<ActivePoint> active_pt, int level, int& row, int& col);
    void removeFromParent(std::shared_ptr<CoarseRegion> coarse_reg);

};
