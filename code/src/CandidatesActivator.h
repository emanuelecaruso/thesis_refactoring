#pragma once
#include "camera.h"
// #include "dso.h"
#include "PointsContainer.h"

class Dso;

class Region{
  public:
    // ********** members **********
    int num_forbid_subregs_;
    int level_;
    int idx_regvecmat_;

    std::vector<std::shared_ptr<CandidateProjected>> candidates_projected_;
    std::vector<std::shared_ptr<Region>> subregions_;


    // ********** constructor **********
    Region(int level, int num_forbid_subregs):
    num_forbid_subregs_(num_forbid_subregs)
    ,level_(level)
    {

    }

    // ********** methods **********
    void pushCandidateProj(std::shared_ptr<CandidateProjected> cand_proj);
    void pushSubreg(std::shared_ptr<Region> subreg);

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
    void reset();
    void clear();
};

class CandprojpresenceMat{
  public:

  // ********** members **********
    unsigned int nrows_;
    unsigned int ncols_;
    bool** mat_;

    // ********** constructor **********
    CandprojpresenceMat(std::shared_ptr<Dso> dso)
    {
      init(dso);
    }

    ~CandprojpresenceMat(){
      for( int i = 0; i < nrows_; i++ ) {
        delete[] mat_[i];
      }
      delete[] mat_;

    }


    // ********** methods **********
    void init(std::shared_ptr<Dso> dso);
    // void clear();
    // void buildLevel0(std::shared_ptr<std::vector<std::shared_ptr<ActivePointProjected>>> v);
    // void updateFromLowerLevel(std::shared_ptr<ActptpresenceMat> lower);


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
              mat_[i][j] = false;
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
      init(dso);
    }

    ~ActptpresenceMatVec(){ }
    // ********** methods **********
    void init(std::shared_ptr<Dso> dso);
    void clear();
    void reset();

};


class RegionsMat{
  public:
    // ********** members **********
      unsigned int nrows_;
      unsigned int ncols_;
      std::shared_ptr<Region>** mat_;

      // ********** constructor **********
      RegionsMat(int nrows, int ncols):
      nrows_(nrows)
      ,ncols_(ncols)
      {
        mat_=new std::shared_ptr<Region>*[nrows];
        for( int i = 0; i < nrows; i++ ) {
          mat_[i] = new std::shared_ptr<Region>[ncols];
          for( int j = 0; j < ncols; j++ ) {
              std::shared_ptr<Region> ptr(nullptr);
              mat_[i][j] = ptr;
          }
        }
      }

      ~RegionsMat(){
        for( int i = 0; i < nrows_; i++ ) {
            delete[] mat_[i];
        }
        delete[] mat_;

      }


      // ********** methods **********
      std::shared_ptr<RegionsMat> getHigherLevel();
      bool checkRegion(int row, int col);
      void clear();
      // bool pushRegion(int row, int col, Region*);

};

class RegMatVec{
  public:
    // ********** members **********

    std::vector<std::shared_ptr<RegionsMat>> regionsmat_vec_;

    // ********** constructor **********
    RegMatVec(std::shared_ptr<Dso> dso){
      init(dso);
    }
    // ********** methods **********
    void init(std::shared_ptr<Dso> dso);
    void clear();

};


class CandidatesActivator{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;
    ActptpresenceMatVec actptpresencemat_vec_;
    RegMatVec regmat_vec_;
    RegVecMat regvec_mat_;
    int num_current_active_points_;
    CandprojpresenceMat candprojpresencemat_;



    // ********** constructor **********
    CandidatesActivator(Dso* dso):
    dso_(dso)
    ,actptpresencemat_vec_(dso_)
    ,regmat_vec_(dso_)
    ,regvec_mat_(this)
    ,num_current_active_points_(0)
    ,candprojpresencemat_(dso_)
    {};

    // ********** methods **********
    void reset();
    // void activateCandidateFromRegion();
    void activateCandidate(std::shared_ptr<CandidateProjected> cand_proj);
    void activateCandidates();
    // void activateCandidates();
    // void collectRegions();
    // void collectRegion(int level);
    // void addKeyframe(bool fixed);
};
