#pragma once
#include "camera.h"
#include "LinSystem.h"
#include "PointsContainer.h"

class Dso;

class MeasTracking : public Meas{
  public:
    // ********** members **********

    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;

    // ********** constructor **********
    MeasTracking(ActivePoint* active_point, CamCouple* cam_couple , int level):
    Meas(active_point, cam_couple, level)
    { }

    // ********** methods **********
    void loadJacobians(ActivePoint* active_point);
};

class LinSysTracking : public LinSys{
  public:
    // ********** members **********
    Eigen::Matrix<float,6,6> H;
    Eigen::Matrix<float,6,1> b;
    Eigen::Matrix<float,6,1> dx;

    // ********** constructor **********
    LinSysTracking(Dso* dso):
    LinSys(dso)
    {
      clear();
    };

    // ********** methods **********
    void addMeasurement( MeasTracking& measurement );
    void updateCameraPose();
    void clear();


};
