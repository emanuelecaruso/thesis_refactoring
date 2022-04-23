#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;

class MeasTracking{
  public:
    // ********** members **********
    bool valid;
    Eigen::Matrix<float,1,6> J_m;
    pixelIntensity error;
    float weight;


    // ********** constructor **********
    MeasTracking(std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple ){
      init(active_point, cam_couple);
    }

    // ********** methods **********
    bool init(std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple);
    Eigen::Matrix<float,1,2> getImageJacobian(pxl& pixel_m, std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, int image_type);
    float getError(pxl& pixel_m, std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, int image_type);
    bool getPixelOfProjectedActivePoint(std::shared_ptr<ActivePoint> active_point, std::shared_ptr<CamCouple> cam_couple, pxl& pixel);

};

class LinSysTracking{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;
    Eigen::Matrix<float,6,6> H;
    Eigen::Matrix<float,6,1> b;

    // ********** constructor **********
    LinSysTracking(std::shared_ptr<Dso> dso):
    dso_(dso)
    {
      H.setZero();
      b.setZero();
    };

    // ********** methods **********
    void addMeasurement( std::shared_ptr<MeasTracking> measurement );
    void updateCameraPose();
  protected:


};
