#include "parameters.h"
#include "environment.h"
#include "dso.h"
#include <stdio.h>
#include <cassert>
#include <memory>

using namespace std;


int main (int argc, char * argv[]) {


  // read arguments
  const std::string dataset_name = argv[1]; // dataset name
  const std::string path_name = "./dataset/"+dataset_name;

  // initialization


  Dso* dso = new Dso( path_name, dataset_name ); // dense mapper and tracker

  dso->startSequential();
  // dtam->test_dso();
  // dtam->test_dso_sequential();
  // dtam->test_optimization_pose();
  // dtam->test_optimization_points();
  // dtam->test_mapping();
  // dtam->test_tracking();

  // dtam->testFeatures();
  // cv::waitKey(0);

  cout << "\nPress Enter to exit"<< endl;
  cin.ignore();
  // // --------------------------------------
  return 1;
}
