#pragma once
#include "camera.h"
#include "epline.h"
#include "CamCouple.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"

class Dso;



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
    void showProjectedActivePoints();

    void sampleCandidates();
    void projectCandidatesOnLastFrame();
    void projectCandidates(std::shared_ptr<CameraForMapping> cam_r, std::shared_ptr<CameraForMapping> cam_m );
    void projectActivePoints(std::shared_ptr<CameraForMapping> cam_r, std::shared_ptr<CameraForMapping> cam_m );

    void trackCandidates();
  protected:
    void trackCandidates(std::shared_ptr<CameraForMapping> keyframe, std::shared_ptr<CameraForMapping> last_keyframe);
    bool trackCandidate(std::shared_ptr<Candidate> cand, std::shared_ptr<CamCouple> cam_couple);
};

class Searcher{
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
    Searcher( std::shared_ptr<EpipolarLine> ep_segment,
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
    float getCostMagn(pxl& pixel);
    bool getPhaseCostContribute(pxl& pixel, float& cost_phase);
    float getStandardDeviation( );
    void updateCand( );


};
