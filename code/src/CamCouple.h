#pragma once
#include "camera.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"
// #include "PointsContainer.h"

class Dso;

class CamCouple{
  public:
    CameraForMapping* cam_r_; // cam to be projected
    CameraForMapping* cam_m_; // cam on which project
    Eigen::Isometry3f T; //cam_r expressed in cam_m


    CamCouple(CameraForMapping* cam_r, CameraForMapping* cam_m):
    cam_r_(cam_r),
    cam_m_(cam_m)
    {
      update();
    }

    void update();
    // EpipolarLine* getEpSegment(Candidate* candidate, int bound_idx);
    // EpipolarLine* getEpSegmentDefaultBounds(float u1, float v1);

    bool getBounds(float u1, float v1, float min_depth, float max_depth, float& bound_low, float& bound_up , bool u_or_v);
    bool getCoord(float u1, float v1, float d1, float& coord, bool u_or_v);
    bool getUv(float u1, float v1, float d1, float& u2, float& v2 );
    bool getD1(float u1, float v1, float& d1, float coord, bool u_or_v);
    bool getD1(float u1, float v1, float& d1, float u2, float v2);
    bool getD2(float u1, float v1, float d1, float& d2);
    bool getSlope(float u1, float v1, float& slope_m);

    void getJrParameters();
    Eigen::Matrix<float,2,1> getJd_(ActivePoint* active_pt);
    Eigen::Matrix<float,2,6> getJm_(ActivePoint* active_pt);
    Eigen::Matrix<float,2,6> getJm_old_(ActivePoint* active_pt);
    Eigen::Matrix<float,2,6> getJr_(ActivePoint* active_pt);

  private:
    Eigen::Matrix3f r;
    Eigen::Vector3f t;
    float f, f2, w, h, w2, h2;
    Eigen::Vector2f cam_r_projected_in_cam_m;

    //Parameters for slope
    float A_s,B_s,C_s, D_s,E_s,F_s;
    //Parameters for bounds
    float A_bu,B_bu,C_bu,D_bu, E_bu,F_bu,G_bu,H_bu;
    float A_bv,B_bv,C_bv,D_bv, E_bv,F_bv,G_bv,H_bv;
    //Parameters for depth
    float A_d, B_d, C_d, D_d;

    // // Jm params
    // float P1, P2, P3, P4;
    // float A1, A2, A3, A4;
    // float B1, B2, B3, B4;
    // float K1, K2, K3, K4, K5, K6, K7, K8, K9, K10;
    // float L1, L2, L3, L4, L5, L6, L7, L8, L9, L10;
    // float Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Z9, Z10;
    // float X1, X2, X3, X4, X5, X6, X7, X8, X9, X10;

    // Jr params
    float C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12;
    float D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12;

    void initJmParameters();
    void initJrParameters();

    void getSlopeParameters();
    void getBoundsParameters();
    void getDepthParameters();

    inline Eigen::Isometry3f getRelativeTransformation(){
      return (*(cam_m_->frame_world_wrt_camera_))*(*(cam_r_->frame_camera_wrt_world_));
    }
};

class CamCoupleContainer{
  public:
    // ********** members **********
    Dso* dso_;
    int type_;
    std::vector<std::vector<CamCouple*>> cam_couple_mat_; // cam_couple_mat_[cam_m][cam_r]


    // ********** constructor **********
    CamCoupleContainer(Dso* dso, int type):
    dso_( dso ),
    type_(type)
    {
      init();
    }

    CamCoupleContainer(Dso* dso, CameraForMapping* cam_r):
    dso_( dso ),
    type_(KF_ON_ALL_KFS)
    {
      init(cam_r);
    }

    ~CamCoupleContainer(){
      for(int i=0; i<cam_couple_mat_.size(); i++){
        for(int j=0; j<cam_couple_mat_[i].size(); j++){
          delete cam_couple_mat_[i][j];
        }
      }
    }

    // ********** methods **********
    void update();
    CamCouple* get(int cam_r_idx, int cam_m_idx);
    void init();
    void init(CameraForMapping* cam_r);

  protected:
    void clear();


};
