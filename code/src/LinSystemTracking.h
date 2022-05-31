#pragma once
#include "camera.h"
#include "LinSystem.h"
#include "PointsContainer.h"

class Dso;

class MeasTracking : public Meas{
  public:
    // ********** members **********

    Eigen::Matrix<float,1,J_SZ> J_m;
    Eigen::Matrix<float,J_SZ,1> J_m_transpose;

    // ********** constructor **********
    MeasTracking(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple , int level):
    Meas(active_point, cam_couple, level)
    { }

    // ********** methods **********
    void loadJacobians(ActivePoint* active_point);
};

class LinSysTracking : public LinSys{
  public:
    // ********** members **********
    Eigen::Matrix<float,J_SZ,J_SZ> H;
    Eigen::Matrix<float,J_SZ,1> b;
    Eigen::Matrix<float,J_SZ,1> dx;

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
