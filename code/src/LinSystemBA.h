#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;


class MeasBA{
  public:
    // ********** members **********
    CamCouple* cam_couple_;
    bool valid_;
    bool occlusion_;
    pxl pixel_;
    float J_d;
    Eigen::Matrix<float,1,6> J_r;
    Eigen::Matrix<float,6,1> J_r_transpose;
    Eigen::Matrix<float,1,6> J_m;
    Eigen::Matrix<float,6,1> J_m_transpose;
    pixelIntensity error;

    // ********** constructor **********
    MeasBA(ActivePoint* active_point, CamCouple* cam_couple, float thresh ):
    valid_(true),
    occlusion_(false),
    cam_couple_(cam_couple)
    {
      init(active_point, cam_couple, thresh);
    }


    // ********** methods **********
    bool init(ActivePoint* active_point, CamCouple* cam_couple, float thresh);
    void loadJacobians(ActivePoint* active_point);
    Eigen::Matrix<float,1,2> getImageJacobian(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type);
    float getError(pxl& pixel_m, ActivePoint* active_point, CamCouple* cam_couple, int image_type);
    bool getPixelOfProjectedActivePoint(ActivePoint* active_point, CamCouple* cam_couple, pxl& pixel);

};

class LinSysBA{
  public:
    // ********** members **********
    Dso* dso_;

    int c_size;
    int p_size;
    Eigen::MatrixXf H_cc;
    Eigen::MatrixXf H_cp;
    Eigen::VectorXf H_pp;
    Eigen::VectorXf b_c;
    Eigen::VectorXf b_p;
    Eigen::VectorXf dx_c;
    Eigen::VectorXf dx_p;

    float chi;

    // ********** constructor **********
    LinSysBA(Dso* dso):
    dso_(dso)
    {
      init();
    };

    // ********** methods **********
    // void addMeasurement( MeasBA& measurement );
    // void updateCameraPose();
    void init();
    void reinitWithNewPoints(int n_points);
    void buildLinearSystem(std::vector<std::vector<MeasBA*>*>& measurement_vec_vec );
    void updateState();
    void updateCameras();
    void updatePoints();
    bool visualizeH();
    // void clear();
  protected:
    float getWeight(float error);
    float addMeasurement(MeasBA* measurement, int p_idx);

};
