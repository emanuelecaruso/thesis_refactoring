#include "CamCouple.h"
#include "dso.h"


Eigen::Isometry3f CamCouple::getRelativeTransformation(){
  Eigen::Isometry3f relative_transf = (*(cam_m_->frame_world_wrt_camera_))*(*(cam_r_->frame_camera_wrt_world_));
  reorthonormalization(relative_transf);
  return relative_transf;
}

void CamCouple::getTLin(){
  if(first_est_jac){

    lin_computed_=true;

    Eigen::Isometry3f T_m;
    if(!cam_m_->cam_data_for_ba_->has_prior_){
      T_m = *(cam_m_->frame_world_wrt_camera_);
    }
    else{
      T_m = cam_m_->cam_data_for_ba_->frame_world_wrt_camera_0_;
    }
    Eigen::Isometry3f T_r;
    if(!cam_r_->cam_data_for_ba_->has_prior_){
      T_r = *(cam_r_->frame_camera_wrt_world_);
    }
    else{
      T_r = cam_r_->cam_data_for_ba_->frame_camera_wrt_world_0_;
    }

    Eigen::Isometry3f relative_transf = T_m*T_r;
    reorthonormalization(relative_transf);

    T0 = relative_transf;

  }
  else{
    T0=T;
  }

  r0=T0.linear();
  t0=T0.translation();

  A0_bu=4*f*r0(0,0) + 2*r0(2,0)*w;
  B0_bu=4*f*r0(0,1) + 2*r0(1,2)*w;
  C0_bu=4*f2*r0(0,2) - r0(2,0)*w2 - 2*f*h*r0(0,1) - 2*f*r0(0,0)*w + 2*f*r0(2,2)*w - h*r0(1,2)*w;
  D0_bu=4*t0(0)*f2 + 2*t0(2)*w*f;
  E0_bu=4*r0(2,0);
  F0_bu=4*r0(1,2);
  G0_bu=4*f*r0(2,2) - 2*h*r0(1,2) - 2*r0(2,0)*w;
  H0_bu= 4*f*t0(2);

  A0_bv=4*f*r0(1,0) + 2*h*r0(2,0);
  B0_bv=4*f*r0(1,1) + 2*h*r0(1,2);
  C0_bv=4*f2*r0(1,2) - h2*r0(1,2) - 2*f*h*r0(1,1) + 2*f*h*r0(2,2) - 2*f*r0(1,0)*w - h*r0(2,0)*w;
  D0_bv=4*t0(1)*f2 + 2*h*t0(2)*f;
  E0_bv=4*r0(2,0);
  F0_bv=4*r0(1,2);
  G0_bv=4*f*r0(2,2) - 2*h*r0(1,2) - 2*r0(2,0)*w;
  H0_bv=4*f*t0(2);
}

void CamCouple::update(){
  T=getRelativeTransformation();
  f=cam_r_->cam_parameters_->lens;
  f2=f*f;
  w=cam_r_->cam_parameters_->width;
  w2=w*w;
  h=cam_r_->cam_parameters_->height;
  h2=h*h;
  r=T.linear();
  t=T.translation();

  cam_m_->projectCam(cam_r_, cam_r_projected_in_cam_m);

  getSlopeParameters();
  getBoundsParameters();
  getDepthParameters();
}

void CamCouple::getSlopeParameters(){

  A_s=2*r(1,0)*t(2) - 2*r(2,0)*t(1);
  B_s=2*r(1,1)*t(2) - 2*r(1,2)*t(1);
  C_s=2*f*r(1,2)*t(2) - 2*f*r(2,2)*t(1) - h*r(1,1)*t(2) + h*r(1,2)*t(1) - r(1,0)*t(2)*w + r(2,0)*t(1)*w;
  D_s=2*r(0,0)*t(2) - 2*r(2,0)*t(0);
  E_s=2*r(0,1)*t(2) - 2*r(1,2)*t(0);
  F_s=2*f*r(0,2)*t(2) - 2*f*r(2,2)*t(0) - h*r(0,1)*t(2) + h*r(1,2)*t(0) - r(0,0)*t(2)*w + r(2,0)*t(0)*w;
}

void CamCouple::getBoundsParameters(){

  A_bu=4*f*r(0,0) + 2*r(2,0)*w;
  B_bu=4*f*r(0,1) + 2*r(1,2)*w;
  C_bu=4*f2*r(0,2) - r(2,0)*w2 - 2*f*h*r(0,1) - 2*f*r(0,0)*w + 2*f*r(2,2)*w - h*r(1,2)*w;
  D_bu=4*t(0)*f2 + 2*t(2)*w*f;
  E_bu=4*r(2,0);
  F_bu=4*r(1,2);
  G_bu=4*f*r(2,2) - 2*h*r(1,2) - 2*r(2,0)*w;
  H_bu= 4*f*t(2);

  A_bv=4*f*r(1,0) + 2*h*r(2,0);
  B_bv=4*f*r(1,1) + 2*h*r(1,2);
  C_bv=4*f2*r(1,2) - h2*r(1,2) - 2*f*h*r(1,1) + 2*f*h*r(2,2) - 2*f*r(1,0)*w - h*r(2,0)*w;
  D_bv=4*t(1)*f2 + 2*h*t(2)*f;
  E_bv=4*r(2,0);
  F_bv=4*r(1,2);
  G_bv=4*f*r(2,2) - 2*h*r(1,2) - 2*r(2,0)*w;
  H_bv=4*f*t(2);

}


// void CamCouple::getJrParameters(){
//   C1 = f*r(0,0)*r(1,2);
//   C2 = f*r(2,0)*t(0);
//   C3 = f*r(0,0)*t(2);
//   C4 = f*r(0,2)*r(2,0);
//   C5 = f*r(0,0)*r(2,2);
//   C6 = f*r(0,1)*r(2,0);
//   C7 = f*r(1,2)*t(0);
//   C8 = f*r(0,1)*t(2);
//   C9 = f*r(0,2)*r(1,2);
//   C10 = f*r(0,1)*r(2,2);
//   C11 = f*r(2,2)*t(0);
//   C12 = f*r(0,2)*t(2);
//
//   D1 = f*r(1,1)*r(2,0);
//   D2 = f*r(1,0)*r(1,2);
//   D3 = f*r(1,2)*r(2,0);
//   D4 = f*r(1,0)*r(2,2);
//   D5 = f*r(2,0)*t(1);
//   D6 = f*r(1,0)*t(2);
//   D7 = f*r(1,2)*r(1,2);
//   D8 = f*r(1,1)*r(2,2);
//   D9 = f*r(1,2)*t(1);
//   D10 = f*r(1,1)*t(2);
//   D11 = f*r(2,2)*t(1);
//   D12 = f*r(1,2)*t(2);
// }

void CamCouple::getJrParameters(){
  C1 = f*r0(0,0)*r0(1,2);
  C2 = f*r0(2,0)*t0(0);
  C3 = f*r0(0,0)*t0(2);
  C4 = f*r0(0,2)*r0(2,0);
  C5 = f*r0(0,0)*r0(2,2);
  C6 = f*r0(0,1)*r0(2,0);
  C7 = f*r0(1,2)*t0(0);
  C8 = f*r0(0,1)*t0(2);
  C9 = f*r0(0,2)*r0(1,2);
  C10 = f*r0(0,1)*r0(2,2);
  C11 = f*r0(2,2)*t0(0);
  C12 = f*r0(0,2)*t0(2);

  D1 = f*r0(1,1)*r0(2,0);
  D2 = f*r0(1,0)*r0(1,2);
  D3 = f*r0(1,2)*r0(2,0);
  D4 = f*r0(1,0)*r0(2,2);
  D5 = f*r0(2,0)*t0(1);
  D6 = f*r0(1,0)*t0(2);
  D7 = f*r0(1,2)*r0(1,2);
  D8 = f*r0(1,1)*r0(2,2);
  D9 = f*r0(1,2)*t0(1);
  D10 = f*r0(1,1)*t0(2);
  D11 = f*r0(2,2)*t0(1);
  D12 = f*r0(1,2)*t0(2);
}

Eigen::Matrix<float,2,1> CamCouple::getJd_(ActivePoint* active_pt ){
  float u1 = active_pt->uv_.x();
  float v1 = active_pt->uv_.y();
  float invd1 = active_pt->invdepth_;

  float du = -(C0_bu*H0_bu - D0_bu*G0_bu + A0_bu*H0_bu*u1 - D0_bu*E0_bu*u1 + B0_bu*H0_bu*v1 - D0_bu*F0_bu*v1)/pow( (H0_bu*invd1 + G0_bu + E0_bu*u1 + F0_bu*v1) ,2);
  float dv = -(C0_bv*H0_bv - D0_bv*G0_bv + A0_bv*H0_bv*u1 - D0_bv*E0_bv*u1 + B0_bv*H0_bv*v1 - D0_bv*F0_bv*v1)/pow( (H0_bv*invd1 + G0_bv + E0_bv*u1 + F0_bv*v1), 2);

  // float du = -(C_bu*H_bu - D_bu*G_bu + A_bu*H_bu*u1 - D_bu*E_bu*u1 + B_bu*H_bu*v1 - D_bu*F_bu*v1)/pow( (H_bu*invd1 + G_bu + E_bu*u1 + F_bu*v1) ,2);
  // float dv = -(C_bv*H_bv - D_bv*G_bv + A_bv*H_bv*u1 - D_bv*E_bv*u1 + B_bv*H_bv*v1 - D_bv*F_bv*v1)/pow( (H_bv*invd1 + G_bv + E_bv*u1 + F_bv*v1), 2);
  Eigen::Matrix<float,2,1> Jd;

  Jd << du,
        dv;

  float pixels_meter_ratio = active_pt->cam_->cam_parameters_->pixel_meter_ratio;
  Jd *= (pixels_meter_ratio/pow(2,active_pt->level_));

  return Jd;
}

Eigen::Matrix<float,2,6> CamCouple::getJr_(ActivePoint* active_pt ){
  Eigen::Vector3f pb = active_pt->p_incamframe_;
  float pbx2 = pb.x()*pb.x();
  float pby2 = pb.y()*pb.y();
  float pbz2 = pb.z()*pb.z();

  float den=pbx2*pow(r0(2,0),2) + 2*pb.x()*pb.y()*r0(1,2)*r0(2,0) + 2*pb.x()*pb.z()*r0(2,0)*r0(2,2) + 2*pb.x()*r0(2,0)*t0(2) + pby2*pow(r0(1,2),2) + 2*pb.y()*pb.z()*r0(1,2)*r0(2,2) + 2*pb.y()*r0(1,2)*t0(2) + pbz2*pow(r0(2,2),2) + 2*pb.z()*r0(2,2)*t0(2) + pow(t0(2),2);
  // float den=pbx2*pow(r(2,0),2) + 2*pb.x()*pb.y()*r(1,2)*r(2,0) + 2*pb.x()*pb.z()*r(2,0)*r(2,2) + 2*pb.x()*r(2,0)*t(2) + pby2*pow(r(1,2),2) + 2*pb.y()*pb.z()*r(1,2)*r(2,2) + 2*pb.y()*r(1,2)*t(2) + pbz2*pow(r(2,2),2) + 2*pb.z()*r(2,2)*t(2) + pow(t(2),2);

  float num_Jru1 = (C6 - C1)*pb.y() + (C4 - C5)*pb.z() + C2 - C3;
  float Jru1 = num_Jru1/den;

  float num_Jru2 = (C1 - C6)*pb.x() + (C9 - C10)*pb.z() + C7 - C8;
  float Jru2 = num_Jru2/den;

  float num_Jru3 = (C5 - C4)*pb.x() + (C10 - C9)*pb.y() + C11 - C12;
  float Jru3 = num_Jru3/den;

  float num_Jru4 = (C5 - C4)*pb.x()*pb.y() + (C6 - C1)*pb.x()*pb.z() + (C10 - C9)*pby2 + (C11 - C12)*pb.y() + (C10 - C9)*pbz2 + (C8 - C7)*pb.z();
  float Jru4 = num_Jru4/den;

  float num_Jru5 = (C4 - C5)*pbx2 + (C9 - C10)*pb.x()*pb.y() + (C12 - C11)*pb.x() + (C6 - C1)*pb.y()*pb.z() + (C4 - C5)*pbz2 + (C2 - C3)*pb.z();
  float Jru5 = num_Jru5/den;

  float num_Jru6 = (C1 - C6)*pbx2 + (C9 - C10)*pb.x()*pb.z() + (C7 - C8)*pb.x() + (C1 - C6)*pby2 + (C5 - C4)*pb.y()*pb.z() + (C3 - C2)*pb.y();
  float Jru6 = num_Jru6/den;



  float num_Jrv1 = (D1 - D2)*pb.y() + (D3 - D4)*pb.z() + D5 - D6;
  float Jrv1 = num_Jrv1/den;

  float num_Jrv2 = (D2 - D1)*pb.x() + (D7 - D8)*pb.z() + D9 - D10;
  float Jrv2 = num_Jrv2/den;

  float num_Jrv3 = (D4 - D3)*pb.x() + (D8 - D7)*pb.y() + D11 - D12;
  float Jrv3 = num_Jrv3/den;

  float num_Jrv4 = (D4 - D3)*pb.x()*pb.y() + (D1 - D2)*pb.x()*pb.z() + (D8 - D7)*pby2 + (D11 - D12)*pb.y() + (D8 - D7)*pbz2 + (D10 - D9)*pb.z();
  float Jrv4 = num_Jrv4/den;

  float num_Jrv5 = (D3 - D4)*pbx2 + (D7 - D8)*pb.x()*pb.y() + (D12 - D11)*pb.x() + (D1 - D2)*pb.y()*pb.z() + (D3 - D4)*pbz2 + (D5 - D6)*pb.z();
  float Jrv5 = num_Jrv5/den;

  float num_Jrv6 = (D2 - D1)*pbx2 + (D7 - D8)*pb.x()*pb.z() + (D9 - D10)*pb.x() + (D2 - D1)*pby2 + (D4 - D3)*pb.y()*pb.z() + (D6 - D5)*pb.y();
  float Jrv6 = num_Jrv6/den;


  Eigen::Matrix<float,2,6> Jr;
  Jr << Jru1, Jru2, Jru3, Jru4, Jru5, Jru6,
        Jrv1, Jrv2, Jrv3, Jrv4, Jrv5, Jrv6;

  float pixels_meter_ratio = active_pt->cam_->cam_parameters_->pixel_meter_ratio;
  Jr *= (pixels_meter_ratio/pow(2,active_pt->level_));

  assert(Jr.allFinite());
  return Jr;
}

Eigen::Matrix<float,2,6> CamCouple::getJm_(ActivePoint* active_pt ){
  Eigen::Vector3f pb = T0*active_pt->p_incamframe_;
  float pbx2 = pb.x()*pb.x();
  float pby2 = pb.y()*pb.y();
  float pbz2 = pb.z()*pb.z();

  float Jmu1 =f/pb.z();
  float Jmu2 =0;
  float Jmu3 =((-f)*pb.x())/pbz2;
  float Jmu4 =((-f)*pb.x()*pb.y())/pbz2;
  float Jmu5 =(f*pbx2 + f*pbz2)/pbz2;
  float Jmu6 =((-f)*pb.y())/pb.z();

  float Jmv1 = 0;
  float Jmv2 = f/pb.z();
  float Jmv3 = ((-f)*pb.y())/pbz2;
  float Jmv4 = (- f*pby2 - f*pbz2)/pbz2;
  float Jmv5 = (f*pb.x()*pb.y())/pbz2;
  float Jmv6 = (f*pb.x())/pb.z();

  Eigen::Matrix<float,2,6> Jm;
  Jm << Jmu1, Jmu2, Jmu3, Jmu4, Jmu5, Jmu6,
        Jmv1, Jmv2, Jmv3, Jmv4, Jmv5, Jmv6;

  float pixels_meter_ratio = active_pt->cam_->cam_parameters_->pixel_meter_ratio;
  Jm *= (pixels_meter_ratio/pow(2,active_pt->level_));

  return Jm;
}


Eigen::Matrix<float,2,6> CamCouple::getJm_old_(ActivePoint* active_pt ){

  CameraForMapping* cam_r = active_pt->cam_;
  CameraForMapping* cam_m = cam_m_;

  float pixels_meter_ratio = active_pt->cam_->cam_parameters_->pixel_meter_ratio;
  Eigen::Matrix3f K = *(cam_m->K_);
  float coeff = pixels_meter_ratio/pow(2,active_pt->level_);

  // variables
  Eigen::Vector2f uv_m_0;
  pxl pixel_m_0;
  pxl pixel_m;
  Eigen::Vector2f uv_m;
  Eigen::Vector3f p_incamframe = active_pt->p_incamframe_;


  Eigen::Vector3f point_m_0= (*(cam_m->frame_world_wrt_camera_))*(active_pt->p_);
  Eigen::Vector3f point_m=   (*(cam_m->frame_world_wrt_camera_))*(active_pt->p_);

  Eigen::Vector3f p_proj_0 = K*point_m_0;
  // Eigen::Vector3f p_proj_0 = K*point_m_0;


  uv_m_0 = p_proj_0.head<2>()*(1./p_proj_0.z());

  cam_m->projectPointInCamFrame( point_m_0, uv_m_0 );
  cam_m->uv2pixelCoords(uv_m_0, pixel_m_0, active_pt->level_);

  cam_m->projectPointInCamFrame( point_m, uv_m );
  cam_m->uv2pixelCoords(uv_m, pixel_m, active_pt->level_);

  Eigen::Matrix<float,2,3> J_first_;

  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;

  proj_jacobian << 1./p_proj_0.z(), 0, -p_proj_0.x()/pow(p_proj_0.z(),2),
                   0, 1./p_proj_0.z(), -p_proj_0.y()/pow(p_proj_0.z(),2);


  J_first_ = coeff*((proj_jacobian)*K);
  assert(J_first_.allFinite());


  Eigen::Matrix<float, 3,6> state_jacobian ;

  state_jacobian << 1, 0, 0,  0             ,  point_m_0.z()  , -point_m_0.y(),
                    0, 1, 0, -point_m_0.z() ,  0              ,  point_m_0.x(),
                    0, 0, 1,  point_m_0.y() , -point_m_0.x()  ,  0         ;

  Eigen::Matrix<float,2,6> Jm = J_first_*state_jacobian;


  return Jm;
}

void CamCouple::getDepthParameters(){
  A_d=r(2,0)/f;
  B_d=r(1,2)/f;
  C_d=r(2,2) - (r(2,0)*w)/(2*f) - (h*r(1,2))/(2*f);
  D_d=t(2);
}

bool CamCouple::getSlope(float u1, float v1, float& slope_m){
  slope_m=(A_s*u1+B_s*v1+C_s)/(D_s*u1+E_s*v1+F_s);
  // assert(!std::isnan(slope_m));
  if (std::isnan(slope_m) || std::isinf(slope_m)){
    return false;
  }
  return true;
}

bool CamCouple::getBounds(float u1, float v1, float min_depth, float max_depth, float& bound_low, float& bound_up , bool u_or_v){

  bool c1 = getCoord(u1, v1, min_depth, bound_low, u_or_v);
  bool c2 = getCoord(u1, v1, max_depth, bound_up, u_or_v);
  if (!c1 || !c2)
    return false;
  return true;
}

bool CamCouple::getCoord(float u1, float v1, float d1, float& coord, bool u_or_v){
  // u2
  if (u_or_v){
    coord=(A_bu*u1*d1+B_bu*v1*d1+C_bu*d1+D_bu)/(E_bu*u1*d1+F_bu*v1*d1+G_bu*d1+H_bu);
  }
  // v2
  else{
    coord=(A_bv*u1*d1+B_bv*v1*d1+C_bv*d1+D_bv)/(E_bv*u1*d1+F_bv*v1*d1+G_bv*d1+H_bv);
  }
  assert(!std::isnan(coord));
  if (std::isnan(coord) || std::isinf(coord)){
    return false;
  }
  return true;
}

bool CamCouple::getUv(float u1, float v1, float d1, float& u2, float& v2 ){
  bool u2_valid = getCoord(u1,v1,d1,u2,true);
  bool v2_valid = getCoord(u1,v1,d1,v2,false);
  return (u2_valid&&v2_valid);

}

bool CamCouple::getD1(float u1, float v1, float& d1, float coord, bool u_or_v){
  // u2
  if (u_or_v){
    d1=((-H_bu)*coord + D_bu)/(E_bu*coord*u1 + F_bu*coord*v1 + G_bu*coord + (-A_bu)*u1 + (-B_bu)*v1 - C_bu);

  }
  // v2
  else{
    d1=((-H_bv)*coord + D_bv)/(E_bv*coord*u1 + F_bv*coord*v1 + G_bv*coord + (-A_bv)*u1 + (-B_bv)*v1 - C_bv);
  }
  assert(std::isfinite(d1));
  if (std::isnan(d1) || std::isinf(d1) ){
    return false;
  }
  return true;

}

bool CamCouple::getD1(float u1, float v1, float& d1, float u2, float v2){
  float slope = (v2-v1)/(u2-u1);
  bool u_or_v = (slope<1 && slope>-1);
  bool out;
  // u2
  if (u_or_v){
    out = getD1( u1, v1, d1, u2, u_or_v);
  }
  // v2
  else{
    out = getD1( u1, v1, d1, v2, u_or_v);
  }

  return out;

}


bool CamCouple::getD2(float u1, float v1, float d1, float& d2){
  d2=(A_d*u1*d1+B_d*v1*d1+C_d*d1+D_d);
  // assert(!std::isnan(d2));
  if (std::isnan(d2) || std::isinf(d2) ){
    return false;
  }
  return true;
}

bool CamCouple::reprojection(const Eigen::Vector2f& uv1, float d1, Eigen::Vector2f& uv2, float& d2){
  Eigen::Vector3f p;
  cam_r_->pointAtDepth(uv1, d1, p);
  cam_m_->projectPoint(p, uv2, d2 );
}

// //
// // EpipolarLine* CamCouple::getEpSegment(Candidate* candidate, int bound_idx){
// //
// //   float u1=candidate->uv_.x();
// //   float v1=candidate->uv_.y();
// //
// //   float min_depth=candidate->bounds_->at(bound_idx).first;
// //   float max_depth=candidate->bounds_->at(bound_idx).second;
// //
// //   float slope_m, bound_up, bound_low;
// //   getSlope(u1, v1, slope_m);
// //   if (std::isnan(slope_m) || std::isinf(slope_m))
// //     return nullptr;
// //
// //   bool u_or_v = (slope_m<1 && slope_m>-1);
// //
// //   bool boundsnotnan = getBounds(u1, v1, min_depth, max_depth, bound_low, bound_up, u_or_v);
// //   if (!boundsnotnan)
// //     return nullptr;
// //
// //   EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m, candidate->level_);
// //
// //
// //   return ep_seg;
// //
// // }
// //
// //
// //
// // EpipolarLine* CamCouple::getEpSegmentDefaultBounds(float u1, float v1){
// //
// //   float min_depth=cam_r_->cam_parameters_->min_depth;
// //   float max_depth=cam_r_->cam_parameters_->max_depth;
// //
// //   float slope_m, bound_up, bound_low;
// //   getSlope(u1, v1, slope_m);
// //
// //   bool u_or_v = (slope_m<1 && slope_m>-1);
// //
// //   getBounds(u1, v1, min_depth, max_depth, bound_low, bound_up, u_or_v);
// //
// //   EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m );
// //
// //   return ep_seg;
// //
// // }



void CamCoupleContainer::init(){

  // cam_couple_mat_[cam_m][cam_r]
  int n_active_kfs = dso_->cameras_container_->keyframes_active_.size();
  if(type_==ALL_KFS_ON_LAST){
    CameraForMapping* cam_m = dso_->frame_current_;
    cam_couple_mat_.resize(1);  //
    cam_couple_mat_[0].resize(n_active_kfs-1);
    // iterate through keyframes (except last)
    for( int i=0; i<n_active_kfs-1 ; i++){
      CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[i];

      // cam couple keyframe with last keyframe
      cam_couple_mat_[0][i]= std::make_shared<CamCouple>( cam_r, cam_m ) ;
    }
  }

  // cam_couple_mat_[cam_m][cam_r]
  else if(type_==ALL_KFS_ON_ALL_KFS){
    cam_couple_mat_.resize(n_active_kfs);  //
    // iterate through keyframes (except last)
    for( int i=0; i<n_active_kfs ; i++){
      CameraForMapping* cam_m = dso_->cameras_container_->keyframes_active_[i];
      cam_couple_mat_[i].resize(n_active_kfs-1);
      for( int j=0; j<n_active_kfs-1 ; j++){

        if (i==j){
          continue;
        }

        CameraForMapping* cam_r = dso_->cameras_container_->keyframes_active_[j];

        std::shared_ptr<CamCouple> cam_couple = std::make_shared<CamCouple>( cam_r, cam_m );
        cam_couple->getJrParameters();
        cam_couple_mat_[i][j]= cam_couple;
      }
    }
  }
}

void CamCoupleContainer::init(CameraForMapping* cam_r, bool get_jr ){

  // cam_couple_mat_[cam_r][cam_m]
  int n_active_kfs = dso_->cameras_container_->keyframes_active_.size();
  cam_couple_mat_.resize(1);  //
  cam_couple_mat_[0].resize(n_active_kfs);
  // iterate through keyframes (except last)
  for( int i=0; i<n_active_kfs ; i++){
    CameraForMapping* cam_m = dso_->cameras_container_->keyframes_active_[i];

    if(cam_m==cam_r)
      cam_couple_mat_[0][i]= nullptr;

    // cam couple keyframe with last keyframe
    cam_couple_mat_[0][i]= std::make_shared<CamCouple>( cam_r, cam_m );
    if(get_jr)
      cam_couple_mat_[0][i]->getJrParameters();
  }


}

void CamCoupleContainer::update(){
  for(int i=0; i<cam_couple_mat_.size(); i++){
    for(int j=0; j<cam_couple_mat_[i].size(); j++){
      cam_couple_mat_[i][j]->update();
    }
  }
}

std::shared_ptr<CamCouple> CamCoupleContainer::get(int cam_r_idx, int cam_m_idx){
  if(type_!=KF_ON_ALL_KFS){
    assert(cam_m_idx>=0 && cam_m_idx<cam_couple_mat_.size());
    assert(cam_r_idx>=0 && cam_r_idx<cam_couple_mat_[cam_m_idx].size());
    return cam_couple_mat_[cam_m_idx][cam_r_idx];
  }
  else{
    assert(cam_r_idx>=0 && cam_r_idx<cam_couple_mat_.size());
    assert(cam_m_idx>=0 && cam_m_idx<cam_couple_mat_[cam_r_idx].size());
    return cam_couple_mat_[cam_r_idx][cam_m_idx];
  }

}
