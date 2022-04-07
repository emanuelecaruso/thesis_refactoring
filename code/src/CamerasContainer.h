#pragma once
#include "camera.h"
#include "parameters.h"
#include <vector>
#include <memory>

class Dso;
class CameraForMapping;

class CamerasContainer{
  public:
    // ********** members **********
    Dso* dso_;

    // cam vectors
    std::vector<std::shared_ptr<CameraForMapping>> frames_;
    std::vector<std::shared_ptr<CameraForMapping>> keyframes_active_;
    std::vector<std::shared_ptr<CameraForMapping>> keyframes_to_marginalize_;
    std::vector<std::shared_ptr<CameraForMapping>> keyframes_marginalized_;

    // ********** constructor **********
    CamerasContainer(Dso* dso):
    dso_(dso){}
    ~CamerasContainer(){}

    // ********** methods **********
    // frame management functions
    void addFrame(std::shared_ptr<Camera> frame);
    void addActiveKeyframe(std::shared_ptr<CameraForMapping> keyframe);
    void addKeyframeToMarginalize(std::shared_ptr<CameraForMapping> keyframe);
    void addKeyframeMarginalized(std::shared_ptr<CameraForMapping> keyframe);
    void removeFrame(std::shared_ptr<Camera> frame);
    void removeActiveKeyframe(std::shared_ptr<CameraForMapping> keyframe);
    void removeKeyframeToMarginalize(std::shared_ptr<CameraForMapping> keyframe);
    void removeKeyframeMarginalized(std::shared_ptr<CameraForMapping> keyframe);

  protected:
    // check functions
    bool checkFrame(std::shared_ptr<CameraForMapping> frame);
    bool checkActiveKeyframe(std::shared_ptr<CameraForMapping> frame);
    bool checkKeyframeToBeMarginalized(std::shared_ptr<CameraForMapping> frame);
    bool checkMarginalizedKeyframe(std::shared_ptr<CameraForMapping> frame);
};
