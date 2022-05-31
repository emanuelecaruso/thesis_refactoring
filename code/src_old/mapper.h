#pragma once
#include "camera.h"
#include "epline.h"

class Dso; //forward declaration
class ActivePoint;

class CamCouple{
  public:
    bool take_fixed_point_;
    CameraForMapping* cam_r_; // cam to be projected
    CameraForMapping* cam_m_; // cam on which project
    Eigen::Isometry3f T; //cam_r expressed in cam_m
    Eigen::Matrix3f r;
    Eigen::Vector3f t;
    float f, f2, w, h, w2, h2;
    Eigen::Vector2f cam_r_projected_in_cam_m;

    CamCouple(CameraForMapping* cam_r, CameraForMapping* cam_m, bool take_fixed_point=false):
    take_fixed_point_(take_fixed_point),
    cam_r_(cam_r),
    cam_m_(cam_m),
    T(getRelativeTransformation(take_fixed_point)),
    f(cam_r->cam_parameters_->lens), f2(f*f), w(cam_r->cam_parameters_->width),
    w2(w*w), h(cam_r->cam_parameters_->height), h2(h*h)
    {
      r=T.linear();
      t=T.translation();

      cam_m->projectCam(cam_r, cam_r_projected_in_cam_m);

      getSlopeParameters();
      getBoundsParameters();
      getDepthParameters();
    }
    EpipolarLine* getEpSegment(Candidate* candidate, int bound_idx);
    EpipolarLine* getEpSegmentDefaultBounds(float u1, float v1);
    // EpipolarLine* getEpSegmentGt(Candidate* candidate);
    // void compareEpSegmentWithGt(Candidate* candidate);
    // void showEpSegment(Candidate* candidate);

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
    Eigen::Matrix<float,2,6> getJr_(ActivePoint* active_pt);

  private:
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


    // //Parameters for Jru
    // float Jru1_A, Jru1_B, Jru1_C, Jru1_D, Jru1_E, Jru1_F, Jru1_G, Jru1_H, Jru1_I ,Jru1_L, Jru1_M, Jru1_N, Jru1_O;
    // float Jru2_A, Jru2_B, Jru2_C, Jru2_D, Jru2_E, Jru2_F, Jru2_G, Jru2_H, Jru2_I ,Jru2_L, Jru2_M, Jru2_N, Jru2_O;
    // float Jru3_A, Jru3_B, Jru3_C, Jru3_D, Jru3_E, Jru3_F, Jru3_G, Jru3_H, Jru3_I ,Jru3_L, Jru3_M, Jru3_N, Jru3_O;
    // float Jru4_A, Jru4_B, Jru4_C, Jru4_D, Jru4_E, Jru4_F, Jru4_G, Jru4_H, Jru4_I, Jru4_L, Jru4_M, Jru4_N, Jru4_O, Jru4_P, Jru4_Q, Jru4_R;
    // float Jru5_A, Jru5_B, Jru5_C, Jru5_D, Jru5_E, Jru5_F, Jru5_G, Jru5_H, Jru5_I, Jru5_L, Jru5_M, Jru5_N, Jru5_O, Jru5_P, Jru5_Q, Jru5_R;
    // float Jru6_A, Jru6_B, Jru6_C, Jru6_D, Jru6_E, Jru6_F, Jru6_G, Jru6_H, Jru6_I, Jru6_L, Jru6_M, Jru6_N, Jru6_O, Jru6_P, Jru6_Q, Jru6_R;
    //
    // //Parameters for Jrv
    // float Jrv1_A, Jrv1_B, Jrv1_C, Jrv1_D, Jrv1_E, Jrv1_F, Jrv1_G, Jrv1_H, Jrv1_I ,Jrv1_L, Jrv1_M, Jrv1_N, Jrv1_O;
    // float Jrv2_A, Jrv2_B, Jrv2_C, Jrv2_D, Jrv2_E, Jrv2_F, Jrv2_G, Jrv2_H, Jrv2_I ,Jrv2_L, Jrv2_M, Jrv2_N, Jrv2_O;
    // float Jrv3_A, Jrv3_B, Jrv3_C, Jrv3_D, Jrv3_E, Jrv3_F, Jrv3_G, Jrv3_H, Jrv3_I ,Jrv3_L, Jrv3_M, Jrv3_N, Jrv3_O;
    // float Jrv4_A, Jrv4_B, Jrv4_C, Jrv4_D, Jrv4_E, Jrv4_F, Jrv4_G, Jrv4_H, Jrv4_I, Jrv4_L, Jrv4_M, Jrv4_N, Jrv4_O, Jrv4_P, Jrv4_Q, Jrv4_R;
    // float Jrv5_A, Jrv5_B, Jrv5_C, Jrv5_D, Jrv5_E, Jrv5_F, Jrv5_G, Jrv5_H, Jrv5_I, Jrv5_L, Jrv5_M, Jrv5_N, Jrv5_O, Jrv5_P, Jrv5_Q, Jrv5_R;
    // float Jrv6_A, Jrv6_B, Jrv6_C, Jrv6_D, Jrv6_E, Jrv6_F, Jrv6_G, Jrv6_H, Jrv6_I, Jrv6_L, Jrv6_M, Jrv6_N, Jrv6_O, Jrv6_P, Jrv6_Q, Jrv6_R;

    void initJmParameters();
    void initJrParameters();

    void getSlopeParameters();
    void getBoundsParameters();
    void getDepthParameters();

    inline Eigen::Isometry3f getRelativeTransformation(bool take_fixed_point){
      if(!take_fixed_point)
        return (*(cam_m_->frame_world_wrt_camera_))*(*(cam_r_->frame_camera_wrt_world_));
      else
        return (*(cam_m_->frame_world_wrt_camera_0_))*(*(cam_r_->frame_camera_wrt_world_0_));

    }
};

class Mapper{

  public:
    Mapper(Dso* dtam, Params* parameters):
      dtam_(dtam),
      parameters_(parameters)
      {};

    void selectNewCandidates();

    void trackExistingCandidates(bool take_gt_points=false, bool debug_mapping=false);



  private:
    Dso* const dtam_;
    Params* const parameters_;


    bool initializeCandidates(CameraForMapping* cam_r,
                            CameraForMapping* cam_m, int& current_r_idx);

    float computeStandardDeviation(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple, Eigen::Vector2f& uv_min, float pixel_width);
    void updateBoundsAndGetSD(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple, float& standard_deviation);
    void updateBounds(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple);

    CandidateProjected* projectCandidateAndUpdateCandInvdepth(Candidate* candidate, CamCouple* cam_couple, EpipolarLine* ep_line , Eigen::Vector2f uv_curr );
    CandidateProjected* projectCandidate(Candidate* candidate, CamCouple* cam_couple );
    CandidateProjected* projectCandidate(Candidate* candidate, CamCouple* cam_couple, EpipolarLine* ep_line );
    void trackExistingCandidates_( bool debug_mapping);
    void trackExistingCandidatesGT();
};
