# Emanuele Caruso - A direct VO based on Direct Sparse Odometry

A direct visual odometry algorithm based on

Direct Sparse Odometry
J. Engel, D. Cremers
2016 

## INSTALL  (Ubuntu 20.04 LTS)

The following packages are required for the installation

###### CMake 3.22:
    
```
sudo apt install cmake
```

###### Ninja 1.10.1

```
sudo apt install ninja-build
```

###### Python 3

```
apt-get install python3-dev
```

###### Eigen3 stable version 3.4.0:

download stable version 3.4.0 and install it with cmake

go to http://eigen.tuxfamily.org
download tar.gz file, extract it and open a terminal inside the folder


```
mkdir build
cd build
cmake ..
make .
sudo make install
```


###### OpenCV version 4.2:

be sure that CUDA toolkit is already installed (previous step)

```
sudo apt install libopencv-dev python3-opencv
```

to verify the version installed

```
python3 -c "import cv2; print(cv2.__version__)"
```

###### (Optional) Blender 3.90

for generating new datasets, download and install Blender 3.90

https://download.blender.org/release/


## COMPILATION

just run build_dtam.sh script inside directory: "scripts"

to change number of threads involved during the compilation, edit build_dtam.sh
by default 8 threads are used: make -j8


## EXECUTION


just run run_dtam.sh script inside directory: "scripts"

to run different datasets, edit run_dtam.sh by changing "dataset_name" variable


## GENERATE DATASET


To generate new Datasets, just a blender scene saved as .blend file is needed inside "blender_scenes" directory

To generate a set of cameras, the "create_cameras_in_scene.py" python script should be runned inside blender (scripting section)

Notice that for different set of cameras, "create_cameras_in_scene.py" has to be edited

After the file .blend is saved, to generate the dataset for dtam, run "generate_dataset.sh" inside directory: "scripts"

inside "generate_dataset.sh" you can choose:

    the dataset by assigning to "dataset_name" variable the name of the .blend file

    the engine (CYCLES = pathtracer) and the number of samples for CYCLES
# thesis
