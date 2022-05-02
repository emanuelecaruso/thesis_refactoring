#pragma once
#include "camera.h"
#include "PointsContainer.h"


// vector for each level of coarse active points
class ActptsCoarseVec{
  public:
    // ********** members **********
    std::vector<std::vector<ActivePoint*>> vec_;

    // ********** constructor **********
    ActptsCoarseVec(int levels){
      vec_.resize(levels);
    }

    // ********** methods **********
    inline void push(ActivePoint* active_pt){
      vec_[active_pt->level_-1].push_back(active_pt);
    }

    inline void clear(){
      for(int i=0; i<vec_.size() ; i++)
        vec_[i].clear();
    }
};

// coarse region
class CoarseRegion : public Point{
  public:
    // ********** members **********
    CoarseRegion* parent_;
    std::vector<CoarseRegion*> subregions_;
    std::vector<ActivePoint*> active_pts_;


    // ********** constructor **********
    CoarseRegion(CameraForMapping* cam, pxl& pixel, int level):
    Point(cam, pixel, level)
    ,parent_(nullptr)
    {}

    // ********** methods **********
    void updateFromActivePts();
    void updateFromSubregions();
    ActivePoint* generateCoarseActivePoint();

};

// matrix of coarse regions
class CoarseRegionMat{
  public:
    // ********** members **********
    CoarseRegion*** mat_;
    unsigned int nrows_;
    unsigned int ncols_;

    // ********** constructor **********
    CoarseRegionMat(int nrows, int ncols){
      mat_=new CoarseRegion**[nrows];
      for( int i = 0; i < nrows; i++ ) {
        mat_[i] = new CoarseRegion*[ncols];
        for( int j = 0; j < ncols; j++ ) {
          CoarseRegion* ptr(nullptr);
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
    std::vector<CoarseRegionMat*> coarseregionsmat_vec_;


    // ********** constructor **********
    CoarseRegionMatVec(PointsContainer* points_container, int levels){
      init(points_container, levels);
    }

    // ********** methods **********
    void init(PointsContainer* points_container, int levels);

};

class CoarseRegions{
  public:
    // ********** members **********
    PointsContainer* points_container_;
    int levels_;
    CoarseRegionMatVec coarseregionsmat_vec_;
    ActptsCoarseVec actptscoarse_vec_;
    std::vector<std::vector<CoarseRegion*>> coarseregions_vec_;


    // ********** constructor **********
    CoarseRegions(PointsContainer* points_container, int levels):
    points_container_(points_container)
    ,levels_(levels)
    ,coarseregionsmat_vec_(points_container, levels)
    ,actptscoarse_vec_(levels)
    {
      coarseregions_vec_.resize(levels);
    }

    // ********** methods **********
    void generateCoarseActivePoints();
    void addActivePoint(ActivePoint* active_pt);
    void removeActivePoint(ActivePoint* active_pt);
    std::vector<ActivePoint*>& getCoarseActivePoints(int level);
    void showCoarseLevel(int level);

  protected:
    void getLevelRowCol(ActivePoint* active_pt, int level, int& row, int& col);
    void removeFromParent(CoarseRegion* coarse_reg);

};
