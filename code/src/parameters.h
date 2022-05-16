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

// debug parameters
extern bool debug_initialization;
extern bool debug_mapping;
extern bool debug_tracking;
extern bool debug_optimization;
extern bool use_spectator;

// code parameters
extern bool do_marginalization;
extern bool first_2_active_kfs_fixed;
extern bool marg_pts_in_marg_kf;

extern bool take_gt_poses;
extern bool take_gt_points;
extern int guess_type;
extern int opt_norm;
extern int image_id;
extern bool test_marginalization;
extern bool active_all_candidates;
extern bool show_spectator;
extern bool get_current_frame;

// candidate selection
extern int candidate_level;
extern int coarsest_level;
extern int reg_level;
extern float grad_threshold;
extern int num_candidates;
extern float der_threshold;

// mapping
extern float cost_threshold;
extern float g_th;

extern float var_threshold;

// keyframe selection
extern int num_active_keyframes;
extern float flow_dist_threshold;
extern float percentage_marg_pts_threshold;

// optimization
extern int max_iterations_ba;
extern int max_num_active_points;
extern float intensity_coeff;
extern float gradient_coeff;
extern float phase_coeff;
extern float damp_point_invdepth;
extern float huber_threshold;
extern float sat_threshold;
extern float chi_occlusion_threshold;
extern float total_error_thresh;
extern float occlusion_valid_ratio_thresh;
extern float valid_ratio_thresh;

// tracking
extern int max_iterations_ls;
extern float variance;
extern int robustifier_dofs;
extern float ratio_for_convergence;
extern float conv_threshold;


//  video streaming
extern int end_frame;
extern int fps;

// initializer parameters
extern int n_corners;
extern float quality_level;
extern float min_distance;
extern float err_threshold;
extern int size_window;
extern float confidence;
extern float ransacReprojThreshold;

// spectator parameters
extern int spec_resolution_x;
extern int spec_resolution_y;
extern float spec_width;
extern float spec_lens;
extern float spec_min_depth;
extern float spec_max_depth;
extern float spec_distance;
extern float rendered_cams_size;
