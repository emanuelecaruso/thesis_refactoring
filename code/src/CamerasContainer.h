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
    std::vector<CameraForMapping*> frames_;
    std::vector<CameraForMapping*> keyframes_active_;
    std::vector<CameraForMapping*> keyframes_to_marginalize_;
    std::vector<CameraForMapping*> keyframes_marginalized_;

    // ********** constructor **********
    CamerasContainer(Dso* dso):
    dso_( dso ){}
    ~CamerasContainer(){}

    // ********** methods **********
    // frame management functions
    void addFrame(Camera* frame);
    void addActiveKeyframe(CameraForMapping* keyframe);
    void addKeyframeToMarginalize(CameraForMapping* keyframe);
    void addKeyframeMarginalized(CameraForMapping* keyframe);
    void removeFrame(Camera* frame);
    void removeActiveKeyframe(CameraForMapping* keyframe);
    void removeKeyframeToMarginalize(CameraForMapping* keyframe);
    void removeKeyframeMarginalized(CameraForMapping* keyframe);

    CameraForMapping* getLastActiveKeyframe();
    CameraForMapping* getSecondLastFrame();
    CameraForMapping* getThirdLastFrame();

  protected:
    // check functions
    bool checkFrame(CameraForMapping* frame);
    bool checkActiveKeyframe(CameraForMapping* frame);
    bool checkKeyframeToBeMarginalized(CameraForMapping* frame);
    bool checkMarginalizedKeyframe(CameraForMapping* frame);
};
