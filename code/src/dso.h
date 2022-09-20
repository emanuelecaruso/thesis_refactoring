#pragma once
#include "matplotlibcpp.h"
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
class PoseNormError;

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
    cv::VideoWriter video_spec_;
    cv::VideoWriter video_proj_;

    // flags for dso phases
    bool first_frame_to_set_ = true;
    bool to_initialize_ = true;

    // flow
    CameraForMapping* frame_current_;
    int frame_current_idx_=-1;

    // tests
    std::vector<float> invdepth_errors;
    std::vector<float> gradients;
    std::vector<int> num_iterations_ls;


    // thread synchronization
    mutable std::mutex mu_frame_;

    std::condition_variable frame_updated_;



    // ********** constructor **********
    Dso(const std::string& path_name, const std::string& dataset_name):
      environment_( new Environment(path_name, dataset_name))
      ,cam_parameters_(new CamParameters(environment_->cam_parameters_))
      ,cameras_container_( new CamerasContainer(this) )
      ,bundle_adj_( new BundleAdj(this))
      ,points_handler_( new PointsHandler(this) )
      ,tracker_( new Tracker(this) )
      ,keyframe_handler_( new KeyframeHandler(this) )
      ,initializer_( new Initializer(this) )
      ,candidates_activator_( new CandidatesActivator(this) )
      ,spectator_( new Spectator(this, white) )
      ,video_spec_("videos/"+environment_->dataset_name_+"_spec.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 10, cv::Size(spectator_->spectator_params_->resolution_x,spectator_->spectator_params_->resolution_y))
      ,video_proj_("videos/"+environment_->dataset_name_+"_proj.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 10, cv::Size(cam_parameters_->resolution_x,cam_parameters_->resolution_y))
      {};

    // Dso();
    ~Dso(){
      delete environment_;
      delete cameras_container_;
      delete bundle_adj_;
      delete points_handler_;
      delete tracker_;
      delete keyframe_handler_;
      delete initializer_;
      delete candidates_activator_;
      delete spectator_;
    }

    // ********** methods **********
    void startSequential();
    void startParallel();
    void startParallelFull();

    // tests
    void testInvdepths();

    // thread synchronization
    void waitForNewFrame();

    // tests
    float getMeanInvdepthErr();
    void plotInvdepthAccWithDer();
    PoseNormError getTotalPosesNormError();

  protected:
    void updateCamerasFromEnvironment(); // load camera objects at fps rate
    bool loadFrameCurrent();
    void setFirstKeyframe();
    bool initialize();
    bool doDso();
    void saveJsonForBlender();


  private:

    // note weak_ptr
};
