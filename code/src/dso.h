#pragma once
#include "CamerasContainer.h"
#include "environment.h"
#include <memory>
#include <condition_variable>

class CameraForMapping;

class Dso{
  public:
    // ********** members **********
    std::shared_ptr<Params> parameters_;
    std::shared_ptr<Environment> environment_;
    CamerasContainer cameras_container_;

    // thread synchronization
    mutable std::mutex mu_frame_;

    std::condition_variable frame_updated_;



    // ********** constructor **********
    Dso(std::shared_ptr<Environment> environment, std::shared_ptr<Params> parameters):
      parameters_(parameters)
      ,environment_(environment)
      ,cameras_container_()
      {};

    // Dso();
    ~Dso(){}

    // ********** methods **********
    void startSequential();

  protected:
    void updateCamerasFromEnvironment(); // load camera objects at fps rate
};
