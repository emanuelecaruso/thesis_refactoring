#include "parameters.h"
#include "environment.h"
#include "dso.h"
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
  debug_gt_error= false;
  take_gt_poses=true;
  take_gt_points=false;
  take_gt_initialization=true;
  remove_occlusions_gt=true;
  max_iterations_ba=0;
  max_iterations_ls=0;
  occlusion_coeff=0;
  collect_invdepth_errs=true;
  use_spectator=false;
  // read arguments
  const std::vector<std::string> datasets = {
    // "bunny_1",
    // "bunny_2",
    // "bunny_3",
    // "bunny_4",
    "bunny_5",
    // "courtyard_1",
    "courtyard_2",
    // "courtyard_3",
    // "courtyard_4",
    // "courtyard_5",
    // "cube_1",
    // "cube_2",
    "cube_3",
    // "cube_4",
    // "cube_5",
    "spaceship_1",
    // "spaceship_2",
    // "spaceship_3",
    // "spaceship_4",
    // "spaceship_5"
  };
  std::string path =  "./dataset/";


  std::vector<float> invdepth_errors;
  std::vector<float> gradients;
  float avg_err=0;
  for( int i=0; i<datasets.size(); i++ ){
  // for( std::string dataset : datasets ){
    std::string dataset = datasets[i];
    Dso dso( path+dataset, dataset ); // dense mapper and tracker
    dso.invdepth_errors=invdepth_errors;
    dso.gradients=gradients;

    dso.startSequential();

    invdepth_errors=dso.invdepth_errors;
    gradients=dso.gradients;


    float curr_avg = dso.getMeanInvdepthErr();
    avg_err+=curr_avg;
    std::cout << "Average invdepth err: " << curr_avg/datasets.size() << std::endl;
    if(i==datasets.size()-1){

      std::cout << "SIZEEE " << dso.gradients.size() << std::endl;
      std::cout << "AVERAGE INVDEPTH ERR: " << avg_err/datasets.size() << std::endl;
      dso.plotInvdepthAccWithDer();
    }

  }

  // dso->startParallel();
  // dso->startParallelFull();

  cout << "\nPress Enter to exit"<< endl;
  cin.ignore();
  // // --------------------------------------
  return 1;
}
