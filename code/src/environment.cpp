#include "environment.h"
#include "defs.h"
#include "json.hpp"
#include "utils.h"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
using json = nlohmann::json;


std::vector<Camera*>* Environment::loadCameraVector(const std::string& path_name, const std::string& dataset_name, int end_frame){

  std::string json_path = path_name+"/"+dataset_name+".json";
  std::string data_path = path_name+"/data.txt";
  std::string times_path = path_name+"/times.txt";
  std::string images_path = path_name+"/calibrated";

  if(file_exists(json_path))
    return loadCameraVectorBlender(path_name, dataset_name, end_frame);
  else if(file_exists(data_path) && dir_exists(images_path) )
    return loadCameraVectorTUM(path_name, dataset_name, end_frame);
  else
    throw std::invalid_argument( "non valid dataset" );

}


std::vector<Camera*>* Environment::loadCameraVectorBlender(const std::string& path_name, const std::string& dataset_name, int end_frame){

  std::vector<Camera*>* camera_vector = nullptr;

  const char* path_name_ = path_name.c_str(); // dataset name

  std::string complete_path = path_name+"/"+dataset_name+".json";

  struct stat info;
  if( stat( path_name_, &info ) != 0 )
    return camera_vector;
  else if( info.st_mode & S_IFDIR )
    {}
  else
    return camera_vector;



  // read a JSON file
  std::ifstream i(complete_path);
  json j;
  i >> j;


  auto cameras = j.at("cameras");

  camera_vector = new std::vector<Camera*>;

  int count =0;

  for (json::iterator it = cameras.begin(); it != cameras.end(); ++it) {

    if (count>end_frame)
      break;

    std::string name=it.key();


    struct stat info_;
    std::string path_rgb_=(path_name+"/rgb_"+name+".png");
    const char* path_rgb = path_rgb_.c_str(); // dataset name
    if( stat( path_rgb, &info_ ) != 0 )
      continue;

    nlohmann::basic_json<>::value_type f;
    int frame;
    try{
      frame= it.value().at("frame");
      f= it.value().at("pose");
    } catch (std::exception& e) {
      std::string error = ": missing values in json file for cameras";
      std::cout << error << std::endl;
      return camera_vector;
    };


    struct stat info__;
    std::string path_depth_=(path_name+"/depth_"+name+".exr");
    const char* path_depth = path_depth_.c_str(); // dataset name
    if( stat( path_depth, &info__ ) != 0 ){
      Camera* camera( new Camera(name,cam_parameters_, f, path_rgb_ ));
      camera_vector->push_back(camera);
      sharedCoutDebug(camera->name_ + " added in env");
      count++;
    }
    else{
      Camera* camera( new Camera(name,cam_parameters_, f, path_rgb_, path_depth_));
      camera_vector->push_back(camera);
      sharedCoutDebug(camera->name_ + " added in env");
      count++;
    }

  }
  return camera_vector;

}


std::vector<Camera*>* Environment::loadCameraVectorTUM(const std::string& path_name, const std::string& dataset_name, int end_frame){

    std::vector<Camera*>* camera_vector = nullptr;

    std::string data_path = path_name+"/data.txt";
    std::string times_path = path_name+"/times.txt";
    std::string images_path = path_name+"/calibrated";

    int count =0;

    std::fstream newfile;
    newfile.open(times_path);  // open a file to perform write operation using file object
    if (newfile.is_open()){ //checking whether the file is open
      camera_vector = new std::vector<Camera*>;

      std::string line;
      while(getline(newfile, line)){ //read data from file object and put it into string.
        if (count>end_frame)
          break;
        std::vector <std::string> tokens; // store the string in vector
        split_str (line, ' ', tokens); // call function to split the string
        std::string cam_name = tokens.front();
        float time = std::stof(tokens.back());

        Camera* camera = new Camera(cam_name,cam_parameters_, images_path+"/"+cam_name+".jpg",time );
        camera_vector->push_back(camera);
        sharedCoutDebug(camera->name_ + " added in env");
        count++;
      }
    }


    return camera_vector;
}

CamParameters* Environment::loadCamParameters(const std::string& path_name, const std::string& dataset_name){
  std::string json_path = path_name+"/"+dataset_name+".json";
  std::string data_path = path_name+"/data.txt";
  std::string times_path = path_name+"/times.txt";
  std::string images_path = path_name+"/calibrated";

  if(file_exists(json_path))
    return loadCamParametersBlender(path_name, dataset_name);
  else if(file_exists(data_path) && dir_exists(images_path) )
    return loadCamParametersTUM(path_name, dataset_name);
  else
    throw std::invalid_argument( "non valid dataset" );


}

CamParameters* Environment::loadCamParametersBlender(const std::string& path_name, const std::string& dataset_name){

    CamParameters* cam_parameters_out = nullptr;

    const char* path_name_ = path_name.c_str(); // dataset name

    std::string complete_path = path_name+"/"+dataset_name+".json";

    struct stat info;
    if( stat( path_name_, &info ) != 0 )
    {
      printf( "ERROR: Dataset NOT found: %s \n", path_name_ );
      exit(1);
      // return cam_parameters_out;
    }
    else if( info.st_mode & S_IFDIR )
    {
      printf( "Dataset found: %s \n",path_name_ );
    }
    else
    {
      printf( "ERROR: %s Is not a directory\n", path_name_ );
      exit(1);
      // return cam_parameters_out;
    }



    // read a JSON file
    std::ifstream i(complete_path);
    json j;
    i >> j;


    auto environment = j.at("environment");
    try{
      const float lens = environment.at("lens");
      const float max_depth = environment.at("max_depth");
      const float min_depth = environment.at("min_depth");
      const float width = environment.at("width");
      const int resolution_x = environment.at("resolution_x");
      const int resolution_y = environment.at("resolution_y");

      cam_parameters_out = new CamParameters(
        resolution_x, resolution_y, width,
        lens, lens, min_depth, max_depth);

    } catch (std::exception& e) {
      std::string error = ": missing values in json file for environment";
      std::cout << error << std::endl;
      return cam_parameters_out;
    };

    return cam_parameters_out;

}

CamParameters* Environment::loadCamParametersTUM(const std::string& path_name, const std::string& dataset_name){

  CamParameters* cam_parameters_out = nullptr;

  std::vector<std::string> data_tokens{
    "res_x", "res_y", "fx", "fy", "cx", "cy", "pxlmtrratio", "mindepth", "maxdepth"
  };
  std::vector<float> cam_data(9);
  int count = 0;

  std::string data_path = path_name+"/data.txt";

  std::fstream newfile;
  newfile.open(data_path);  // open a file to perform write operation using file object
  if (newfile.is_open()){ //checking whether the file is open
    std::string line;
    while(getline(newfile, line)){ //read data from file object and put it into string.

      std::vector <std::string> tokens; // store the string in vector
      split_str (line, ' ', tokens); // call function to split the string
      assert(tokens.front()==data_tokens[count]);
      cam_data[count]=std::stof(tokens.back());
      count++;
    }
  }
  cam_parameters_out = new CamParameters(
    cam_data[0],
    cam_data[1],
    cam_data[2],
    cam_data[3],
    cam_data[4],
    cam_data[5],
    cam_data[6],
    cam_data[7],
    cam_data[8]
    );



  return cam_parameters_out;

}


void Environment::debugAllCameras(bool show_imgs) const {

  std::cout << "DEBUGGING ALL CAMERAS:" << std::endl;
  std::cout << "camera vector size: " << camera_vector_->size() << std::endl;

  for(Camera* camera : *camera_vector_){
    camera->printMembers();
    if (show_imgs){
      camera->showRGB();
      camera->showDepthMap();
    }
  }
  waitkey(0);
}
