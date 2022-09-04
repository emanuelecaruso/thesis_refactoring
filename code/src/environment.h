#pragma once
#include "camera.h"

class Environment{
  public:

    const std::string dataset_name_;
    const int fps_;
    const int coarsest_level_;
    const CamParameters* cam_parameters_;
    const std::vector<Camera*>* camera_vector_; // vector containing pointers to camera objects
    // int num_current_frames_;


    Environment(const std::string& path_name, const std::string& dataset_name):
    dataset_name_(dataset_name),
    fps_(fps),
    coarsest_level_(coarsest_level),
    cam_parameters_(loadCamParameters( path_name, dataset_name)),
    camera_vector_(loadCameraVector( path_name, dataset_name, end_frame))
    // num_current_frames_(0)
    { };

    ~Environment(){
      // for(Camera* cam : *camera_vector_)
      //   delete cam;
      delete camera_vector_;
      delete cam_parameters_;
    }

    void debugAllCameras(bool show_imgs=false) const;

  private:
    std::vector<Camera*>* loadCameraVector(const std::string& path_name, const std::string& dataset_name, int end_frame);
    std::vector<Camera*>* loadCameraVectorBlender(const std::string& path_name, const std::string& dataset_name, int end_frame);
    std::vector<Camera*>* loadCameraVectorTUM(const std::string& path_name, const std::string& dataset_name, int end_frame);

    CamParameters* loadCamParameters(const std::string& path_name, const std::string& dataset_name);
    CamParameters* loadCamParametersBlender(const std::string& path_name, const std::string& dataset_name);
    CamParameters* loadCamParametersTUM(const std::string& path_name, const std::string& dataset_name);

    // double saveState(std::string path_name, Camera_cpu* camera_cpu);

};
