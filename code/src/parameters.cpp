#include "defs.h"
#include "parameters.h"

// debug parameters
bool debug_initialization=false;
bool debug_mapping=false;
bool debug_tracking=true;
bool debug_optimization= true;
bool debug_gt_error= false;
int debug_start_frame = 200;
bool use_spectator= true;


// code parameters
bool free_mem = true;
bool do_marginalization = true;
bool first_2_active_kfs_fixed = false;
bool marg_pts_in_marg_kf = false;
bool first_est_jac = false;
bool take_gt_poses=false;
bool take_gt_points=false;
bool take_gt_initialization=false;
int guess_type=POSE_CONSTANT;
// int guess_type=VELOCITY_CONSTANT;
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
float grad_threshold=0.02;
int num_candidates=2000;


// keyframe selection
int num_active_keyframes=7;
float flow_dist_threshold=6;
float percentage_marg_pts_threshold= 0.01;
// float percentage_marg_pts_threshold= 0.5;


// optimization
int max_iterations_ba=10;
int max_num_active_points=4000;
float intensity_coeff_ba= 1;
float gradient_coeff_ba= 0;
float g_th_intensity_ba= 2;
float g_th_gradient_ba= 2;
float coeff_sum_ba=intensity_coeff_ba+gradient_coeff_ba;
float damp_cam= 0;
float damp_point_invdepth= 1000;
// float damp_point_invdepth= 10e3;
// float damp_point_invdepth= FLT_MAX;
float damp_exposure= 10;
float lambda_a= 10;
float lambda_b= 10;
float huber_threshold=coeff_sum_ba*0.05;
// float sat_threshold=0.04;
float sat_threshold=coeff_sum_ba*111111;
float occlusion_thres_intensity=0.1;
float occlusion_thres_gradient=0.1;
// float total_error_thresh = 0.015;
float total_error_thresh = coeff_sum_ba*1111;
float occlusion_valid_ratio_thresh= 0.4;
// float occlusion_valid_ratio_thresh= 0.8;
float valid_ratio_thresh= 0.2;



// tracking
int coarsest_level= candidate_level+3; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// int reg_level=candidate_level_+3;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
int max_iterations_ls=100;
float variance= 0.1;
int robustifier_dofs=1;
float ratio_for_convergence= 0.1;
// float conv_threshold= 0.00001;
float conv_threshold= coeff_sum_ba*0.01;


// mapping
// float cost_threshold=0.7;
// float var_threshold= 0.1;
float var_threshold= 1;
// float der_threshold=0.0001;
float der_threshold=0;
float intensity_coeff_mapping= 1;
float gradient_coeff_mapping= 0.5;
float phase_coeff_mapping= 1./(4.*PI);
float g_th_intensity_mapping= 2;
float g_th_gradient_mapping= 2;


//  video streaming
int end_frame=1000;
int fps=30;


// initializer parameters
int n_corners= 2000;
float quality_level= 0.001;
float min_distance= 10;
float err_threshold= 5;
int size_window= 21;
float confidence= 0.999;
float ransacReprojThreshold= 1;
float world_scale_default = 0.2;
float flow_dist_threshold_init = 6;


// spectator parameters
float spec_upscaling = 2;
float spec_distance= 2;
float rendered_cams_size= 0.01;
