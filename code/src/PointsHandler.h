#pragma once
#include "camera.h"
#include "epline.h"
#include "CamCouple.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"

class PointsHandler{
  public:

    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    PointsHandler(Dso* dso ):
    dso_(dso)
    {};


    // ********** methods **********
    void showCandidates();
    void showProjectedCandidates();
    void showActivePoints();
    void showCoarseActivePoints(int level);
    void showProjectedActivePoints(const std::string& name);
    void showProjectedActivePoints();

    void sampleCandidates();
    void projectCandidatesOnLastFrame();
    void projectActivePointsOnLastFrame();
    void generateCoarseActivePoints();
    void projectCandidates(std::shared_ptr<CameraForMapping> cam_r, std::shared_ptr<CameraForMapping> cam_m );
    void projectActivePoints(std::shared_ptr<CameraForMapping> cam_r, std::shared_ptr<CameraForMapping> cam_m );

    void trackCandidates(bool groundtruth);
  protected:
    void trackCandidates(std::shared_ptr<CameraForMapping> keyframe, std::shared_ptr<CameraForMapping> last_keyframe);
    void trackCandidatesGroundtruth(std::shared_ptr<CameraForMapping> keyframe);
    bool trackCandidate(std::shared_ptr<Candidate> cand, std::shared_ptr<CamCouple> cam_couple);
};

class CandTracker{
  public:
    // ********** members **********
    std::shared_ptr<EpipolarLine> ep_segment_;
    std::shared_ptr<Candidate> cand_;
    std::shared_ptr<CamCouple> cam_couple_;
    std::shared_ptr<Params> parameters_;
    float phase_m;
    pixelIntensity magn_m;
    Eigen::Vector2f uv_;
    pxl pixel_;

    // ********** constructor **********
    CandTracker( std::shared_ptr<EpipolarLine> ep_segment,
              std::shared_ptr<Candidate> cand,
              std::shared_ptr<CamCouple> cam_couple,
              std::shared_ptr<Params> parameters
            ):
              ep_segment_(ep_segment),
              cand_(cand),
              cam_couple_(cam_couple),
              parameters_(parameters)
              {};


    // ********** methods **********
    bool searchMin( );
    float getCostMagn(pxl& pixel_m);
    bool getPhaseCostContribute(pxl& pixel_m, Eigen::Vector2f& uv_m, float& cost_phase);
    float getStandardDeviation( );
    void updateCand( );


};
