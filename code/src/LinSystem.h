#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;

class MeasTracking{
  public:
    // ********** members **********
    bool valid_;
    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;
    pixelIntensity error;

    // ********** constructor **********
    MeasTracking(ActivePoint* active_point, CamCouple* cam_couple ):
    valid_(true){
      init(active_point, cam_couple);
    }

    // ********** methods **********
    bool init(ActivePoint* active_point, CamCouple* cam_couple);
    Eigen::Matrix<float,1,2> getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type);
    float getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type);
    bool getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel);

};

class LinSysTracking{
  public:
    // ********** members **********
    Dso* dso_;
    Eigen::Matrix<float,6,6> H;
    Eigen::Matrix<float,6,1> b;
    Eigen::Matrix<float,6,1> dx;
    pixelIntensity chi;

    // ********** constructor **********
    LinSysTracking(Dso* dso):
    dso_(dso)
    {
      clear();
    };

    // ********** methods **********
    void addMeasurement( MeasTracking* measurement );
    void updateCameraPose();
    void clear();
  protected:
    float getWeight(float error);


};
