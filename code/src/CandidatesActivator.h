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

    std::vector<CandidateProjected*> candidates_projected_;
    std::vector<Region*> subregions_;
    Region* parent_;

    // ********** constructor **********
    Region(int level, int num_forbid_subregs):
    num_forbid_subregs_(num_forbid_subregs)
    ,level_(level)
    {

    }

    // ********** methods **********
    void pushCandidateProj(CandidateProjected* cand_proj);
    void pushSubreg(Region* subreg);

};

class CandidatesActivator;

class RegVecMat{
  public:
    // ********** members **********
    CandidatesActivator* cand_activator_;
    // Dso* dso_;
    std::vector<std::vector<std::vector<Region*>>> regs_;
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
    CandprojpresenceMat(Dso* dso)
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
    void init(Dso* dso);

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
    void buildLevel0(std::vector<ActivePointProjected*>& v);
    void updateFromLowerLevel(ActptpresenceMat* lower);


};

class ActptpresenceMatVec{
  public:
    // ********** members **********
    Dso* dso_;
    std::vector<ActptpresenceMat*> actptpresencemat_vec_;
    bool updated_ = false;

    // ********** constructor **********
    ActptpresenceMatVec(Dso* dso):
    dso_(dso)
    {
      init(dso);
    }

    ~ActptpresenceMatVec(){ }
    // ********** methods **********
    void init(Dso* dso);
    void clear();
    void reset();

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
        }

        for( int i = 0; i < nrows_; i++ ) {
            delete[] mat_[i];
        }
        delete[] mat_;

      }


      // ********** methods **********
      RegionsMat* getHigherLevel();
      bool checkRegion(int row, int col);
      void clear();
      // bool pushRegion(int row, int col, Region*);

};

class RegMatVec{
  public:
    // ********** members **********

    std::vector<RegionsMat*> regionsmat_vec_;

    // ********** constructor **********
    RegMatVec(Dso* dso){
      init(dso);
    }
    // ********** methods **********
    void init(Dso* dso);
    void clear();

};


class CandidatesActivator{
  public:
    // ********** members **********
    Dso* dso_;
    ActptpresenceMatVec actptpresencemat_vec_;
    RegMatVec regmat_vec_;
    RegVecMat regvec_mat_;
    CandprojpresenceMat candprojpresencemat_;



    // ********** constructor **********
    CandidatesActivator(Dso* dso):
    dso_(dso)
    ,actptpresencemat_vec_(dso_)
    ,regmat_vec_(dso_)
    ,regvec_mat_(this)
    ,candprojpresencemat_(dso_)
    {};

    // ********** methods **********
    void reset();
    // void activateCandidateFromRegion();
    void removeCand(CandidateProjected* cand_proj);
    void activateCandidate(CandidateProjected* cand_proj);
    void activateCandidates();
    void removeEmptyRegion(Region* reg);
    // void activateCandidates();
    // void collectRegions();
    // void collectRegion(int level);
    // void addKeyframe(bool fixed);
};
