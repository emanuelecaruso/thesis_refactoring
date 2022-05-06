#pragma once
#include "camera.h"
#include "CamerasContainer.h"
#include "PointsContainer.h"
#include "initializer.h"
#include "CandidatesActivator.h"
#include "environment.h"
#include "BundleAdj.h"
#include "KeyframeHandler.h"
#include "PointsHandler.h"
#include "Tracker.h"
#include <memory>
#include <condition_variable>
#include "spectator.h"

class CameraForMapping;

class Dso{
  public:
    // ********** members **********


    // main objects
    Environment* environment_;
    CamParameters* cam_parameters_;
    CamerasContainer* cameras_container_;
    PointsHandler* points_handler_;
    BundleAdj* bundle_adj_;
    Tracker* tracker_;
    KeyframeHandler* keyframe_handler_;
    Initializer* initializer_;
    CandidatesActivator* candidates_activator_;
    Spectator* spectator_;

    // flags for dso phases
    bool first_frame_to_set_ = true;
    bool to_initialize_ = true;

    // flow
    CameraForMapping* frame_current_;
    int frame_current_idx_=-1;


    // thread synchronization
    mutable std::mutex mu_frame_;

    std::condition_variable frame_updated_;



    // ********** constructor **********
    Dso(Environment* environment):
      environment_(environment)
      ,cam_parameters_(new CamParameters(environment_->cam_parameters_))
      ,cameras_container_( new CamerasContainer(this) )
      ,bundle_adj_( new BundleAdj(this))
      ,points_handler_( new PointsHandler(this) )
      ,tracker_( new Tracker(this) )
      ,keyframe_handler_( new KeyframeHandler(this) )
      ,initializer_( new Initializer(this) )
      ,candidates_activator_( new CandidatesActivator(this) )
      ,spectator_( new Spectator(this, white) )
      {};

    // Dso();
    ~Dso(){}

    // ********** methods **********
    void startSequential();

    // thread synchronization
    void waitForNewFrame();

  protected:
    void updateCamerasFromEnvironment(); // load camera objects at fps rate
    void loadFrameCurrent();
    void setFirstKeyframe();
    void initialize();
    bool doDso();

  private:

    // note weak_ptr
};
