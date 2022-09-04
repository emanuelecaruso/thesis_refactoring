#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <unistd.h>
#include "opencv2/opencv.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <mutex>
#include "defs.h"

// code parameters
static bool use_fast_cam_coupling_ = false;
static bool debug_initialization_=true;
static bool debug_mapping_=false;
static bool debug_tracking_=true;
static bool debug_optimization_= true;
static bool take_gt_poses_=false;
static bool take_gt_points_=false;
static int guess_type_=POSE_CONSTANT;
// static int guess_type_=VELOCITY_CONSTANT;
static int opt_norm_=HUBER;
// static int opt_norm_=QUADRATIC;
static int test_single_=TEST_ALL;
// static int image_id_=INTENSITY_ID;
static int image_id_=GRADIENT_ID;
// int image_id_=PHASE_ID;
static bool test_marginalization_=false;
static bool all_keyframes_=false;
static bool active_all_candidates_=true;
static bool show_spectator_=true;
static bool get_current_frame_=false;

// candidate selection
static int coarsest_level_= 2; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
static int reg_level_=3;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
static float grad_threshold_=0.06;
static int num_candidates_=700;

// mapping
static float cost_threshold_=0.07;
static float cost_grad_threshold_=0.08;
static float cost_grad_threshold_DSO_=0.01; // for DSO
static int max_num_mins_ = 3;

// keyframe selection
static int num_active_keyframes_=5;
static float flow_dist_threshold_=0.0001;
// static float percentage_marg_pts_threshold_ = 1.1;
static float percentage_marg_pts_threshold_ = 0.1;

// optimization
static int max_iterations_ba_=20;
static int max_num_active_points_=2000;
// static float huber_threshold_=0.02;
static float huber_threshold_=0.07;
// static float huber_threshold_=0.5;
static float chi_occlusion_threshold_=(0.12-huber_threshold_/2);
// static float chi_occlusion_threshold_=1000;
// static float chi_occlusion_threshold_=(pow(0.2,2));
static int max_occlusions_ = num_active_keyframes_/2;
static float intensity_coeff_ = 1;
static float gradient_coeff_ = 0.25;
static float phase_coeff_ = 1./(8.*PI);
static float damp_pose_position_ = 0;
static float damp_pose_orientation_ = 0;
static float damp_point_invdepth_ = 100;

// tracking
static int max_iterations_ls_=7;
static float variance_ = 0.1;
static int robustifier_dofs_=2;
static float ratio_for_convergence_ = 0.001;

//  video streaming
static int end_frame_=120;
static int fps_=30;

// initializer parameters
static int n_corners_ = 1000;
static float quality_level_ = 0.01;
static float min_distance_ = 10;
static float err_threshold_ = 5;
static int size_window_ = 21;
static float confidence_ = 0.999;
static float ransacReprojThreshold_ = 1;

// spectator parameters
const int spec_resolution_x_ = 1366;
const int spec_resolution_y_ = 768;
const float spec_width_ = 0.024;
const float spec_lens_ = 0.035;
const float spec_min_depth_ = 0.01;
const float spec_max_depth_ = 20;
const float spec_distance_ = 5;
const float rendered_cams_size_ = 0.01;


struct Params{
  bool use_fast_cam_coupling=use_fast_cam_coupling_;
  bool debug_initialization=debug_initialization_;
  bool debug_mapping=debug_mapping_;
  bool debug_tracking=debug_tracking_;
  bool debug_optimization=debug_optimization_;
  bool take_gt_poses=take_gt_poses_;
  bool take_gt_points=take_gt_points_;
  int guess_type=guess_type_;
  int opt_norm=opt_norm_;
  int test_single=test_single_;
  int image_id=image_id_;
  bool test_marginalization=test_marginalization_;
  bool all_keyframes=all_keyframes_;
  bool active_all_candidates=active_all_candidates_;
  bool show_spectator=show_spectator_;
  bool get_current_frame=get_current_frame_;


  int coarsest_level=coarsest_level_;
  int reg_level=reg_level_;
  float grad_threshold=grad_threshold_;
  int num_candidates=num_candidates_;

  float cost_threshold=cost_threshold_;
  float cost_grad_threshold=cost_grad_threshold_;
  float cost_grad_threshold_DSO=cost_grad_threshold_DSO_;
  int max_iterations_ba=max_iterations_ba_;
  int max_num_active_points=max_num_active_points_;
  int num_active_keyframes=num_active_keyframes_;
  float flow_dist_threshold=flow_dist_threshold_;
  float percentage_marg_pts_threshold=percentage_marg_pts_threshold_;

  int max_num_mins=max_num_mins_;
  int end_frame=end_frame_;
  int fps=fps_;
  float huber_threshold=huber_threshold_;
  float chi_occlusion_threshold=chi_occlusion_threshold_;
  float max_occlusions=max_occlusions_;

  float intensity_coeff=intensity_coeff_;
  float gradient_coeff=gradient_coeff_;
  float phase_coeff=phase_coeff_;
  float damp_pose_position=damp_pose_position_;
  float damp_pose_orientation=damp_pose_orientation_;
  float damp_point_invdepth=damp_point_invdepth_;

  int max_iterations_ls=max_iterations_ls_;
  float variance=variance_;
  int robustifier_dofs=robustifier_dofs_;
  int ratio_for_convergence=ratio_for_convergence_;
  int n_corners=n_corners_;
  float quality_level=quality_level_;
  float min_distance=min_distance_;
  float err_threshold=err_threshold_;
  float size_window=size_window_;
  float confidence=confidence_;
  float ransacReprojThreshold=ransacReprojThreshold_;

  int spec_resolution_x=spec_resolution_x_;
  int spec_resolution_y=spec_resolution_y_;
  float spec_width=spec_width_;
  float spec_lens=spec_lens_;
  float spec_min_depth=spec_min_depth_;
  float spec_max_depth=spec_max_depth_;
  float spec_distance=spec_distance_;
  float rendered_cams_size=rendered_cams_size_;

  Params(){
    // if (coarsest_level<reg_level){
    //   std::cout << "Error in parameters: coarsest level less than reg_level!" << std::endl;
    //   exit(1);
    // }
  };


};
