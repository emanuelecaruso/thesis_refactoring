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
    void projectCandidates(CameraForMapping* cam_r, CameraForMapping* cam_m );
    void projectActivePoints(CameraForMapping* cam_r, CameraForMapping* cam_m );

    void trackCandidates(bool groundtruth);
  protected:
    void trackCandidates(CameraForMapping* keyframe, CameraForMapping* last_keyframe);
    void trackCandidatesGroundtruth(CameraForMapping* keyframe);
    bool trackCandidate(Candidate* cand, CamCouple* cam_couple);
};

class CandTracker{
  public:
    // ********** members **********
    EpipolarLine& ep_segment_;
    Candidate* cand_;
    CamCouple* cam_couple_;
    Params* parameters_;
    float phase_m;
    pixelIntensity magn_m;
    Eigen::Vector2f uv_;
    pxl pixel_;

    // ********** constructor **********
    CandTracker( EpipolarLine& ep_segment,
              Candidate* cand,
              CamCouple* cam_couple,
              Params* parameters
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
    bool updateCand( );


};
