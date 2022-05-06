#include "defs.h"
#include "parameters.h"

// debug parameters
bool debug_initialization=false;
bool debug_mapping=false;
bool debug_tracking=false;
bool debug_optimization= false;
bool use_spectator= true;

// code parameters
bool take_gt_poses=false;
bool take_gt_points=false;
int guess_type=POSE_CONSTANT;
// int guess_type=VELOCITY_CONSTANT;
// int guess_type=PERS_GUESS;
int opt_norm=HUBER;
// int opt_norm=QUADRATIC;
int image_id=INTENSITY_ID;
// int image_id=GRADIENT_ID;
bool test_marginalization=false;
bool active_all_candidates=true;
bool show_spectator=true;
bool get_current_frame=false;

// candidate selection
int candidate_level= 0;
int reg_level=candidate_level+3;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// float grad_threshold=0.02;
float grad_threshold=0.05;
int num_candidates=2000;

// mapping
float cost_threshold=0.03;
float var_threshold= 1;
float der_threshold=0.001;
// float var_threshold= 1;

// keyframe selection
int num_active_keyframes=7;
// float flow_dist_threshold=0.0005;
float flow_dist_threshold=0.0002;
float percentage_marg_pts_threshold= 0.1;

// optimization
int max_iterations_ba=10;
int max_num_active_points=2000;
float intensity_coeff= 1;
float gradient_coeff= 0.25;
float phase_coeff= 1./(4.*PI);
float damp_point_invdepth= 1;
float huber_threshold=0.02;
// float sat_threshold=0.04;
float sat_threshold=2;
float chi_occlusion_threshold=0.06;
// float occlusion_valid_ratio_thresh= 0.5;
float occlusion_valid_ratio_thresh= 0.8;
float valid_ratio_thresh= 0.2;

// tracking
int coarsest_level= candidate_level+4; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// int reg_level=candidate_level_+3;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
int max_iterations_ls=100;
float variance= 0.1;
int robustifier_dofs=1;
float ratio_for_convergence= 0.1;
float conv_threshold= 0.001;


//  video streaming
int end_frame=160;
int fps=30;

// initializer parameters
int n_corners= 1000;
float quality_level= 0.01;
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
float spec_distance= 5;
float rendered_cams_size= 0.01;
