#pragma once
#include "defs.h"
#include "environment.h"
#include "image.h"
#include "parameters.h"
#include "camera.h"
// #include "dso.h"

class Dso;

class Spectator{
  public:

    // ********** members **********
    std::shared_ptr<Dso> dso_;
    colorRGB background_color_;
    std::shared_ptr<CamParameters> spectator_params_;
    std::shared_ptr<Camera> spectator_cam_;
    std::shared_ptr<Image<colorRGB>> spectator_image_;

    // ********** constructor **********
    Spectator(Dso* dso, const colorRGB& background_color ):
    dso_(dso),
    background_color_(background_color),
    spectator_params_(initCamParams()),
    spectator_cam_(initCam()),
    spectator_image_(initImage())
    {};

    // ********** methods **********
    void spectateDso();
    void renderState();
    void showSpectator();

  private:

    std::shared_ptr<CamParameters> initCamParams();
    std::shared_ptr<Camera> initCam();
    std::shared_ptr<Image<colorRGB>> initImage();


    // void reinitSpectator();
    // void renderPoints();
    // void renderCamsAndKFs();
    //
    // Eigen::Isometry3f getSpectatorPose();
    //
    // bool plotPt(ActivePoint* pt, const colorRGB& color);
    // bool plotPt(Eigen::Vector3f& pt, const colorRGB& color);
    // bool plotPt(ActivePoint* pt, const colorRGB& color, pxl& pixel);
    // bool plotPt(Eigen::Vector3f& pt, const colorRGB& color, pxl& pixel);
    // bool plotLine(pxl& pixel1, pxl& pixel2, const colorRGB& color );
    // bool plotCam(Camera* cam, const colorRGB& color);
};
