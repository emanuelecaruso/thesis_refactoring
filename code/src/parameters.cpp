#include "defs.h"
#include "parameters.h"


// debug parameters
bool debug_initialization=false;
bool debug_mapping=false;
bool debug_mapping_match = false;
bool debug_tracking=false;
bool debug_optimization= false;
bool debug_gt_error= false;
int debug_start_frame = 0;
bool use_spectator= true;

// tests
bool collect_invdepth_errs=false;
bool test_track=false;

// code parameters
bool free_mem = true;
bool adaptive_thresh = true;
bool reverse_tracking = true;
bool magn_mean = true;
bool do_marginalization = true;
bool first_2_active_kfs_fixed = false;
bool marg_pts_in_marg_kf = false;
bool first_est_jac = false;
bool take_gt_poses=false;
bool take_gt_points=false;
bool take_gt_initialization=true;
bool remove_occlusions_gt=false;
bool pyr_type = true;
// int guess_type=POSE_CONSTANT;
int guess_type=VELOCITY_CONSTANT;
// int guess_type=GT_GUESS;
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
int coarsest_lev_magn = candidate_level+3;
int reg_level=candidate_level+5;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// float grad_threshold=0.1;
float grad_threshold=0.005;
int max_num_active_points=1500;
int num_candidates=max_num_active_points;


// keyframe selection
int num_active_keyframes=7;
float flow_dist_threshold=5;
float percentage_marg_pts_threshold= 0.05;
// float percentage_marg_pts_threshold= 0.5;


// optimization
int max_iterations_ba=10;
float intensity_coeff_ba= 0.2;
float gradient_coeff_ba=0.8;
// float intensity_coeff_ba= 0.9;
// float gradient_coeff_ba=0.1;
float mean_magn_coeff_ba = 3;
// float mean_magn_coeff_ba = 0.;
float g_th_intensity_ba = 0;
float g_th_gradient_ba = 0;
float damp_cam= 0;
float damp_point_invdepth= 10000;
float damp_exposure= 0;
float lambda_a= 0;
float lambda_b= 0;
float sat_coeff=1;
float huber_coeff=0.05;
float occlusion_coeff=0.5;
float fixed_thresh = 0.05;
float occlusion_valid_ratio_thresh= 0.5;
// float occlusion_valid_ratio_thresh= 0.6;
float valid_ratio_thresh= 0.2;
float level_error_multiplier= 2;



// tracking
int coarsest_level= candidate_level+3; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
int max_iterations_ls=1000;
float variance= 0.1;
int robustifier_dofs=1;
float ratio_for_convergence= 0.1;
// float conv_threshold= 0.00001;
float conv_threshold= 0.000001;
// float conv_threshold= (intensity_coeff_ba+gradient_coeff_ba)*0.00000000001;
float level_error_multiplier_conv=50;
int coarsest_level_pyr=std::max(coarsest_level,coarsest_lev_magn);
int n_reverse_min = 1;


// mapping
// float cost_threshold=0.7;Bidirectional
// float var_threshold= 1000;
float var_threshold= 1000;
// float der_threshold=0.0001;
float der_threshold=0;

float intensity_coeff_mapping= 0.5;
float gradient_coeff_mapping= 1;
float phase_coeff= (1./(4.*PI));
// float intensity_coeff_mapping= 1;
// float gradient_coeff_mapping=0;
// float phase_coeff=0;
// float intensity_coeff_mapping= 0;
// float gradient_coeff_mapping= 1;
// float phase_coeff= (1./(4.*PI));

float mean_magn_coeff_mapping = 1;
float g_th_intensity_mapping = 0.0;
float g_th_gradient_mapping = 0.0;
int max_pxls_inliers = 5;


//  video streaming
int start_frame=0;
int end_frame=2000;
int fps=30;


// initializer parameters
int n_corners= 5000;
float quality_level= 0.005;
float min_distance= 10;
int size_window= 5;
float confidence= 0.999999;
float ransacReprojThreshold= 1;
float world_scale_default = 0.1;
float flow_dist_threshold_init = 5;
float inliers_ratio = 0.00001;
float reproj_lk_threshold =1;

// spectator parameters
float spec_upscaling = 2;
float spec_distance= 2;
// float spec_distance= 0.5;
float rendered_cams_size= 0.01;
