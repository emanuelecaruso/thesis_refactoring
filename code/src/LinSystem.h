#pragma once
#include "camera.h"
#include "PointsContainer.h"

class Dso;

class Meas{
  public:
    // ********** members **********
    CamCouple* cam_couple_;
    bool valid_;
    bool occlusion_;
    pxl pixel_;
    pixelIntensity error;
    float var_;
    int level_;

    // ********** constructor **********
    Meas(ActivePoint* active_point, CamCouple* cam_couple , int level):
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
    float getWeight();

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
