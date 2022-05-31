#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

class Dso;
class CameraForMapping;

class KeyframeHandler{
  public:
    Dso* dtam_;

    KeyframeHandler(Dso* dtam):
    dtam_(dtam)
    {};

    // bool addKeyframe(bool all_keyframes=false);
    void addFixedKeyrame();
    // bool marginalize_keyframe(bool all_keyframes=false);
    // bool marginalize_keyframe_ba(bool all_keyframes=false);
    //
    // void prepareDataForBA();

  private:
    void pushKeyframeFrontend();
    // void pushKeyframeBundleadj();
    // bool pushKF();
    //
    // void marginalizeKeyframeFrontend(int idx);
    // void marginalizeKeyframeBundleadj(int idx);
    // void marginalizeKeyframe(int idx);
    //
    // bool addKeyframe_select();
    // void marginalizeKeyframeSelect();
    // void marginalizeKeyframeSelect_ba();
    //
    //
    // float getFlowDist();
    // float getPercentuageMarg(CameraForMapping* keyframe);
    // float getScore(CameraForMapping* keyframe);

};
