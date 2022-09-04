#include "parameters.h"
#include "environment.h"
#include "dso.h"
#include "CameraForMapping.h"
#include <stdio.h>
#include <cassert>
#include <memory>

using namespace std;


int main (int argc, char * argv[]) {

  debug_initialization=false;
  debug_mapping=false;
  debug_mapping_match = false;
  debug_tracking=false;
  debug_optimization= false;
  use_spectator=true;
  // debug_gt_error= false;
  // take_gt_poses=false;
  // take_gt_points=false;
  // take_gt_initialization=true;
  // remove_occlusions_gt=false;
  // max_iterations_ba=10;
  // max_iterations_ls=1000;
  // test_track=true;

  // read arguments
  const std::vector<std::string> datasets = {
    "bunny_1",
    "bunny_2",
    "bunny_3",
    "bunny_4",
    "bunny_5",
    "courtyard_1",
    "courtyard_2",
    "courtyard_3",
    "courtyard_4",
    "courtyard_5",
    "cube_1",
    "cube_2",
    "cube_3",
    "cube_4",
    "cube_5",
    "spaceship_1",
    "spaceship_2",
    "spaceship_3",
    "spaceship_4",
    "spaceship_5"
  };
  std::string path =  "./dataset/";

  std::vector<int> num_iterations_ls_tot;
  PoseNormError pose_norm_error_tot = PoseNormError();
  for( int i=0; i<datasets.size(); i++ ){
  // for( std::string dataset : datasets ){
    std::string dataset = datasets[i];
    Dso dso( path+dataset, dataset ); // dense mapper and tracker

    dso.startSequential();

    pose_norm_error_tot += dso.getTotalPosesNormError();

    for( int iterations : dso.num_iterations_ls){
      num_iterations_ls_tot.push_back(iterations);
    }

  }

  pose_norm_error_tot/=datasets.size();
  std::cout << "Total error poses" << std::endl;
  pose_norm_error_tot.print();

  float iterations_sum=0;
  for( int iterations : num_iterations_ls_tot)
    iterations_sum += iterations;

  std::cout << "Total iterations avg " << iterations_sum/num_iterations_ls_tot.size() << std::endl;

  // dso->startParallel();
  // dso->startParallelFull();

  cout << "\nPress Enter to exit"<< endl;
  cin.ignore();
  // // --------------------------------------
  return 1;
}
