#pragma once
#include "camera.h"
#include "dso.h"
#include "PointsContainer.h"

// class Dso;

class Region{
  public:
    // ********** members **********
    int num_act_pts_;
    int level_;
    std::vector<std::shared_ptr<CandidateProjected>> candidates_projected_;


    // ********** constructor **********

    // ********** methods **********

};

class CandidatesActivator;

class RegVecMat{
  public:
    // ********** members **********
    std::shared_ptr<CandidatesActivator> cand_activator_;
    // std::shared_ptr<Dso> dso_;
    std::vector<std::vector<std::vector<std::shared_ptr<Region>>>> regs_;
    bool updated_;

    // ********** constructor **********
    RegVecMat(CandidatesActivator* cand_activator):
    cand_activator_(cand_activator)
    {
      init();
    }

    // ********** methods **********
    void init();
    void update();
    void clear();
};


class ActptpresenceMat{
  public:

  // ********** members **********
    unsigned int nrows_;
    unsigned int ncols_;
    bool** mat_;

    // ********** constructor **********
    ActptpresenceMat(int nrows, int ncols):
    nrows_(nrows)
    ,ncols_(ncols)
    {
      mat_=new bool*[nrows];
      for( int i = 0; i < nrows; i++ ) {
          mat_[i] = new bool[ncols];
          for( int j = 0; j < ncols; j++ ) {
              mat_[i][j] = 0;
          }
      }
    }

    ~ActptpresenceMat(){
      for( int i = 0; i < nrows_; i++ ) {
        delete[] mat_[i];
      }
      delete[] mat_;

    }


    // ********** methods **********
    void clear();
    void buildLevel0(std::shared_ptr<std::vector<std::shared_ptr<ActivePointProjected>>> v);
    void updateFromLowerLevel(std::shared_ptr<ActptpresenceMat> lower);


};

class ActptpresenceMatVec{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;
    std::vector<std::shared_ptr<ActptpresenceMat>> actptpresencemat_vec_;
    bool updated_ = false;

    // ********** constructor **********
    ActptpresenceMatVec(std::shared_ptr<Dso> dso):
    dso_(dso)
    {
      int res_x = dso->frame_current_->cam_parameters_->resolution_x/2;
      int res_y = dso->frame_current_->cam_parameters_->resolution_y/2;
      for(int i=0; i<dso->parameters_->reg_level; i++)
        actptpresencemat_vec_[i] = std::make_shared<ActptpresenceMat>(res_y/pow(2,i),res_x/pow(2,i));
    }
    // ********** methods **********
    void clear();
    void update();

};


class RegionsMat{
  public:
    // ********** members **********
      unsigned int nrows_;
      unsigned int ncols_;
      Region*** mat_;

      // ********** constructor **********
      RegionsMat(int nrows, int ncols):
      nrows_(nrows)
      ,ncols_(ncols)
      {
        mat_=new Region**[nrows];
        for( int i = 0; i < nrows; i++ ) {
            mat_[i] = new Region*[ncols];
            for( int j = 0; j < ncols; j++ ) {
                mat_[i][j] = nullptr;
            }
        }
      }

      ~RegionsMat(){
        for( int i = 0; i < nrows_; i++ ) {
            for( int j = 0; j < ncols_; j++ ) {
                delete mat_[i][j];
            }
            delete[] mat_[i];
        }
        delete[] mat_;

      }


      // ********** methods **********
      std::shared_ptr<RegionsMat> getHigherLevel();
      bool checkRegion(int row, int col);
      // bool pushRegion(int row, int col, Region*);

};

class RegMatVec{
  public:
    // ********** members **********

    std::vector<std::shared_ptr<RegionsMat>> regionsmat_vec_;

    // ********** constructor **********
    RegMatVec(std::shared_ptr<Dso> dso){
      int res_x = dso->frame_current_->cam_parameters_->resolution_x/2;
      int res_y = dso->frame_current_->cam_parameters_->resolution_y/2;
      for(int i=0; i<dso->parameters_->reg_level; i++)
        regionsmat_vec_[i] = std::make_shared<RegionsMat>(res_y/pow(2,i),res_x/pow(2,i));
    }
    // ********** methods **********
    void clear();

};


class CandidatesActivator{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;
    ActptpresenceMatVec actptpresencemat_vec_;
    RegMatVec regmat_vec_;
    RegVecMat regvec_mat_;


    // ********** constructor **********
    CandidatesActivator(Dso* dso):
    dso_(dso)
    ,actptpresencemat_vec_(dso_)
    ,regmat_vec_(dso_)
    ,regvec_mat_(this)
    {};

    // ********** methods **********
    void update();
    // void activateCandidates();
    // void collectRegions();
    // void collectRegion(int level);
    // void addKeyframe(bool fixed);
};
