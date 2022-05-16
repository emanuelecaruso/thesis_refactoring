#pragma once
#include "camera.h"
// #include "dso.h"
#include "PointsContainer.h"

class Dso;

class Region{
  public:
    // ********** members **********
    int row_;
    int col_;
    int level_;
    Region* parent_;
    bool removed_;

    std::vector<Region*> subregions_;
    int num_forbid_subregs_;
    float var_;

    CandidateProjected* cand_proj_;

    // ********** constructor **********
    // create region (leaf) from cand proj
    Region(CandidateProjected* cand_proj):
    row_(trunc(cand_proj->pixel_.y())),
    col_(trunc(cand_proj->pixel_.x())),
    level_(0),
    num_forbid_subregs_(0),
    cand_proj_(cand_proj),
    var_(cand_proj->cand_->invdepth_var_),
    parent_(nullptr),
    removed_(false)
    {}

    // create region from Region old
    Region(Region* reg_old):
    row_(trunc(reg_old->row_/2)),
    col_(trunc(reg_old->col_/2)),
    level_(reg_old->level_+1),
    cand_proj_(nullptr),
    var_(reg_old->var_),
    parent_(nullptr),
    removed_(false)

    {
      subregions_.push_back(reg_old);
    }

    // ********** methods **********


};

class CandidatesActivator;

class RegVecMat{
  public:
    // ********** members **********
    CandidatesActivator* cand_activator_;
    // Dso* dso_;
    std::vector<std::vector<std::vector<Region*>>> regs_;

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
    int i_;
    int j_;


    // ********** constructor **********
    CandidatesActivator(Dso* dso):
    dso_(dso)
    ,actptpresencemat_vec_(dso_)
    ,regmat_vec_(dso_)
    ,regvec_mat_(this)
    {};

    // ********** methods **********
    void clearAll();
    void rebuild();
    bool updateLeafRegion(CandidateProjected* cand_proj);
    Region* updateRegion(Region* reg_old);

    CandidateProjected* getNextCandProj();
    CandidateProjected* getCandProjFromReg( Region* reg);
    void clearParentIfEmpty( Region* reg);

    void removeCand(CandidateProjected* cand_proj);
    void activateCandidate(CandidateProjected* cand_proj);
    void activateCandidates();
    void insertRegInVec(Region* reg, std::vector<Region*>& v );


    void printActPresenceMat(ActptpresenceMat* actptpresencemat);
    void printRegionsMat(RegionsMat* regionsmat);

    // void collectRegions();
    // void collectRegion(int level);
    // void addKeyframe(bool fixed);
};
