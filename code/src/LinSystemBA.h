#pragma once
#include "camera.h"
#include "LinSystem.h"
#include "PointsContainer.h"

class Dso;


class MeasBA : public Meas {
  public:


    // ********** members **********
    Eigen::Matrix<float,1,6> J_r;
    Eigen::Matrix<float,6,1> J_r_transpose;
    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;
    float J_d;

    // ********** constructor **********
    MeasBA(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple ):
    Meas(active_point, cam_couple, active_point->level_)
    { }


    // ********** methods **********
    void loadJacobians(ActivePoint* active_point);

};

class LinSysBA : public LinSysBlocks{
  public:
    // ********** members **********
    Eigen::VectorXf dx_c;
    Eigen::VectorXf dx_p;

    // ********** constructor **********
    LinSysBA(Dso* dso):
    LinSysBlocks(dso)
    {
      init();
    };

    // ********** methods **********

    void init();
    void reinitWithNewPoints(int n_points);
    void buildLinearSystem(std::vector<std::vector<MeasBA*>*>& measurement_vec_vec );
    void updateCameras();
    void updatePoints();
  protected:
    float addMeasurement(MeasBA* measurement, int p_idx);

};

class PriorMeas : public Meas{
  public:
    // ********** members **********
    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;
    float J_d;
    int p_idx_;

    // ********** constructor **********
    PriorMeas(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple):
    Meas( active_point, cam_couple , active_point->level_),
    p_idx_(0)
    {
      if(valid_)
        loadJacobians(active_point, cam_couple);
    }

    // ********** methods **********
    void loadJacobians(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple);
};
