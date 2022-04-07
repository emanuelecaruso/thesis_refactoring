#pragma once
#include "camera.h"
#include "CamerasContainer.h"
#include "PointsContainer.h"
#include "initializer.h"
#include "environment.h"
#include "KeyframeHandler.h"
#include "PointsHandler.h"
#include "Tracker.h"
#include <memory>
#include <condition_variable>

class CameraForMapping;

class Dso : public std::enable_shared_from_this<Dso>{
  public:
    // ********** members **********

    // main objects
    std::shared_ptr<Params> parameters_;
    std::shared_ptr<Environment> environment_;
    std::shared_ptr<CamerasContainer> cameras_container_;
    std::shared_ptr<PointsHandler> points_handler_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<KeyframeHandler> keyframe_handler_;
    std::shared_ptr<Initializer> initializer_;

    // flags for dso phases
    bool first_frame_to_set_ = true;
    bool to_initialize_ = true;

    // useful
    std::shared_ptr<CamParameters> cam_parameters_;

    // flow
    std::shared_ptr<CameraForMapping> frame_current_;
    int frame_current_idx_=-1;


    // thread synchronization
    mutable std::mutex mu_frame_;

    std::condition_variable frame_updated_;



    // ********** constructor **********
    Dso(std::shared_ptr<Environment> environment, std::shared_ptr<Params> parameters):
      parameters_(parameters)
      ,environment_(environment)
      ,cameras_container_( new CamerasContainer(this) )
      ,points_handler_( new PointsHandler(this) )
      ,tracker_( new Tracker(this) )
      ,keyframe_handler_( new KeyframeHandler(this) )
      ,initializer_( new Initializer(this) )
      ,cam_parameters_(environment_->cam_parameters_)
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

  private:

    // note weak_ptr
};
