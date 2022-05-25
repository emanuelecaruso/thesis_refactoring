#include "defs.h"
#include "parameters.h"

// debug parameters
bool debug_initialization=false;
bool debug_mapping=false;
bool debug_tracking=false;
bool debug_optimization= false;
int debug_start_frame = 0;
bool use_spectator= true;

// code parameters
bool do_marginalization = true;
bool first_2_active_kfs_fixed = false;
bool marg_pts_in_marg_kf = false;
bool first_est_jac = false;
bool take_gt_poses=false;
bool take_gt_points=false;
bool take_gt_initialization=false;
// int guess_type=POSE_CONSTANT;
int guess_type=VELOCITY_CONSTANT;
int opt_norm=HUBER;
// int opt_norm=QUADRATIC;
// int image_id=INTENSITY_ID;
int image_id=GRADIENT_ID;
bool test_marginalization=false;
bool active_all_candidates=true;
bool show_spectator=true;
bool get_current_frame=false;

// candidate selection
int candidate_level= 0;
int reg_level=candidate_level+5;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// float grad_threshold=0.1;
float grad_threshold=0.05;
int num_candidates=2000;

// mapping
// float cost_threshold=0.7;
// float g_th=0.02;
float g_th=0.1;
// float var_threshold= 0.1;
float var_threshold= 1;
float der_threshold=0;
float intensity_coeff= 1;
float gradient_coeff= 0.5;
float phase_coeff= 1./(4.*PI);
// float der_threshold=0;

// keyframe selection
int num_active_keyframes=7;
float flow_dist_threshold=0.00001;
// float flow_dist_threshold=0.0005;
float percentage_marg_pts_threshold= 0.01;
// float percentage_marg_pts_threshold= 0.5;

// optimization
int max_iterations_ba=10;
int max_num_active_points=4000;

float intensity_coeff_ba= 0.2;
float gradient_coeff_ba= 1;

float damp_cam= 0;
float damp_point_invdepth= 1000;
// float damp_point_invdepth= 10e3;
// float damp_point_invdepth= FLT_MAX;

float huber_threshold=(intensity_coeff*gradient_coeff)*0.05;
// float sat_threshold=0.04;
float sat_threshold=(intensity_coeff*gradient_coeff)*111111;
float chi_occlusion_threshold=(intensity_coeff*gradient_coeff)*0.1;
// float total_error_thresh = 0.015;
float total_error_thresh = (intensity_coeff*gradient_coeff)*1111;
// float occlusion_valid_ratio_thresh= 0.5;
float occlusion_valid_ratio_thresh= 1;
float valid_ratio_thresh= 0.2;

// tracking
int coarsest_level= candidate_level+3; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// int reg_level=candidate_level_+3;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
int max_iterations_ls=100;
float variance= 0.1;
int robustifier_dofs=1;
float ratio_for_convergence= 0.1;
// float conv_threshold= 0.00001;
float conv_threshold= 0.01;


//  video streaming
int end_frame=120;
int fps=30;

// initializer parameters
int n_corners= 2000;
float quality_level= 0.001;
float min_distance= 10;
float err_threshold= 5;
int size_window= 21;
float confidence= 0.999;
float ransacReprojThreshold= 1;

// spectator parameters
int spec_resolution_x= 1366;
int spec_resolution_y= 768;
float spec_width= 0.024;
float spec_lens= 0.035;
float spec_min_depth= 0.01;
float spec_max_depth= 20;
float spec_distance= 3;
float rendered_cams_size= 0.01;
