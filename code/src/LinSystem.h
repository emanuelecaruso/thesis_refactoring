#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;

class Meas{
  public:
    // ********** members **********
    std::shared_ptr<CamCouple> cam_couple_;
    bool valid_;
    bool occlusion_;
    pxl pixel_;
    pixelIntensity error;
    float var_;
    int level_;
    float weight_;

    // ********** constructor **********
    Meas(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple , int level):
    cam_couple_(cam_couple),
    valid_(true),
    occlusion_(false),
    var_(active_point->invdepth_var_),
    level_(level){
      init(active_point);
    }

    // ********** methods **********
    bool init(ActivePoint* active_point);
    Eigen::Matrix<float,1,2> getImageJacobian( int image_type);
    float getError( ActivePoint* active_point, int image_type);
    bool getPixelOfProjectedActivePoint(ActivePoint* active_point);
    float getWeight(ActivePoint* active_point);
  protected:
    float getErrorIntensity(float z, float z_hat );
    float getErrorGradient(float z, float z_hat );

};

class LinSys{
  public:
    // ********** members **********
    Dso* dso_;
    pixelIntensity chi;

    // ********** constructor **********
    LinSys(Dso* dso):
    dso_(dso)
    { };

    // ********** methods **********

};

class LinSysBlocks : public LinSys{
  public:
    // ********** members **********

    int c_size_;
    int p_size_;
    Eigen::MatrixXf H_cc_;
    Eigen::MatrixXf H_cp_;
    Eigen::VectorXf H_pp_;
    Eigen::VectorXf b_c_;
    Eigen::VectorXf b_p_;
    Eigen::VectorXf omega_;


    // ********** constructor **********
    LinSysBlocks(Dso* dso):
    LinSys(dso)
    { };

    // ********** methods **********
    bool visualizeH();
    bool visualizeB();

    void clear();
    void reset();
    void resize(int c_size, int p_size);
    // void resize( int c_size, int p_siz)


};
