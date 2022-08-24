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


extern bool debug_initialization;
extern bool debug_mapping;
extern bool debug_mapping_match;
extern bool debug_tracking;
extern bool debug_optimization;
extern bool debug_gt_error;
extern int debug_start_frame;
extern bool use_spectator;


extern bool free_mem;
extern bool magn_mean;
extern bool adaptive_thresh;
extern bool do_marginalization;
extern bool first_2_active_kfs_fixed;
extern bool sat_separate;
extern bool marg_pts_in_marg_kf;
extern bool dso_original;
extern bool first_est_jac;
extern bool take_gt_poses;
extern bool take_gt_points;
extern bool take_gt_initialization;
extern bool pyr_type;
extern int guess_type;
extern int opt_norm;
extern int image_id;
extern bool test_marginalization;
extern bool active_all_candidates;
extern bool show_spectator;
extern bool get_current_frame;


extern int candidate_level;
extern int coarsest_lev_magn;
extern int coarsest_level_pyr;
extern int reg_level;
extern float grad_threshold;
extern int num_candidates;


extern int num_active_keyframes;
extern float flow_dist_threshold;
extern float percentage_marg_pts_threshold;


extern int max_iterations_ba;
extern int max_num_active_points;
extern float intensity_coeff_ba;
extern float gradient_coeff_ba;
extern float mean_magn_coeff_ba;
extern float g_th_intensity_ba;
extern float g_th_gradient_ba;
// extern float coeff_sum;
extern float damp_cam;
extern float damp_point_invdepth;
extern float damp_exposure;
extern float lambda_a;
extern float lambda_b;
extern float huber_threshold;
extern float sat_coeff;
extern float occlusion_coeff;
extern float huber_coeff;
extern float fixed_thresh;
extern float total_error_thresh;
extern float occlusion_valid_ratio_thresh;
extern float valid_ratio_thresh;
extern float level_error_multiplier;



extern int coarsest_level;
extern int max_iterations_ls;
extern float variance;
extern int robustifier_dofs;
extern float ratio_for_convergence;
extern float conv_threshold;
extern float level_error_multiplier_conv;


extern float var_threshold;
extern float der_threshold;
extern float intensity_coeff_mapping;
extern float gradient_coeff_mapping;
extern float phase_coeff;
extern float mean_magn_coeff_mapping;
extern float g_th_intensity_mapping;
extern float g_th_gradient_mapping;
extern int max_pxls_inliers;


extern int end_frame;
extern int fps;

extern float intensity_coeff_tracking;
extern float gradient_coefftracking;
extern int n_corners;
extern float quality_level;
extern float min_distance;
extern int size_window;
extern float confidence;
extern float ransacReprojThreshold;
extern float world_scale_default;
extern float flow_dist_threshold_init;
extern float inliers_ratio;
extern float reproj_lk_threshold;


extern float spec_upscaling;
extern float spec_distance;
extern float rendered_cams_size;
