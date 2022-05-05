#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;

class MeasTracking{
  public:
    // ********** members **********
    ActivePoint* active_point_;
    int level_;
    bool valid_;
    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;
    pixelIntensity error;

    // ********** constructor **********
    MeasTracking(ActivePoint* active_point, CamCouple* cam_couple , int level, Dso* dso):
    valid_(true),
    level_(level),
    active_point_(active_point){
      init(active_point, cam_couple, level, dso);
    }

    // ********** methods **********
    bool init(ActivePoint* active_point, CamCouple* cam_couple, int level, Dso* dso);
    Eigen::Matrix<float,1,2> getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int level, Dso* dso, int image_type);
    float getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int level, int image_type);
    bool getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel, int level);

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
    void addMeasurement( MeasTracking& measurement );
    void updateCameraPose();
    void clear();

  protected:
    float getWeight(MeasTracking& measurement);

};
