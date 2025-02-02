#pragma once
#include "camera.h"
#include "epline.h"
#include "CamCouple.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"

class PointsHandler{
  public:

    // ********** members **********
    Dso* dso_;
    int n_cands_to_track_;
    int n_cands_removed_;
    int n_cands_repeptitive_;
    int n_cands_no_clear_min_;
    int n_cands_ep_too_short_;
    int n_cands_no_min_;
    int n_cands_tracked_;
    int n_cands_var_too_high_;
    int n_cands_updated_;

    // ********** constructor **********
    PointsHandler(Dso* dso ):
    n_cands_to_track_(0),
    n_cands_removed_(0),
    n_cands_repeptitive_(0),
    n_cands_no_clear_min_(0),
    n_cands_ep_too_short_(0),
    n_cands_no_min_(0),
    n_cands_var_too_high_(0),
    n_cands_tracked_(0),
    n_cands_updated_(0),
    dso_(dso)
    {};


    // ********** methods **********
    void showCandidates();
    void showProjectedCandidates();
    void showProjectedCandidates(const std::string& name);
    void showActivePoints();
    void showCoarseActivePoints(int level);
    void showProjectedActivePoints(const std::string& name, int i=0);
    void showProjectedActivePoints( int i=0);

    bool sampleCandidates();
    void projectCandidatesOnLastFrame();
    void selfProjectCandidatesOnLastFrame();
    void projectActivePointsOnLastFrame();
    void generateCoarseActivePoints();
    void projectCandidates(CameraForMapping* cam_r, CameraForMapping* cam_m );
    void projectActivePoints(CameraForMapping* cam_r, CameraForMapping* cam_m );

    void trackCandidates(bool groundtruth);
    void trackCandidatesReverse(bool groundtruth);

    void removeOcclusionsInLastKFGrountruth();
  protected:
    void trackCandidates(CameraForMapping* kf1, CameraForMapping* kf2, bool remove=true);
    void trackCandidatesGroundtruth(CameraForMapping* keyframe);
    bool trackCandidate(Candidate* cand, std::shared_ptr<CamCouple> cam_couple);
};

class CandTracker{
  public:
    // ********** members **********
    EpipolarLine& ep_segment_;
    Candidate* cand_;
    std::shared_ptr<CamCouple> cam_couple_;
    float phase_m;
    pixelIntensity magn_m;
    Eigen::Vector2f uv_;
    pxl pixel_;
    float thresh_;
    int n_valid_uvs_;

    // ********** constructor **********
    CandTracker( EpipolarLine& ep_segment,
              Candidate* cand,
              std::shared_ptr<CamCouple> cam_couple
            ):
              ep_segment_(ep_segment),
              cand_(cand),
              cam_couple_(cam_couple),
              n_valid_uvs_(0)
              {};


    // ********** methods **********
    int searchMin();
    float getCostMagn(pxl& pixel_m);
    float getCostIntensity(pxl& pixel_m);
    float getCostGradient(pxl& pixel_m);
    bool getPhaseCostContribute(pxl& pixel_m, Eigen::Vector2f& uv_m, float& cost_phase);
    float getStandardDeviation( );
    bool updateCand( int max_pxls_inliers);


};
