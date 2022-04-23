link_libraries(${OpenCV_LIBS})

##test mapping
#add_executable(test_mapping test_mapping.cpp
#../src/camera.cpp
#../src/image.cpp
#../src/camera.cpp
##../src/renderer.cpp
#../src/environment.cpp
#../src/dtam.cpp
#../src/utils.cpp
#../src/epline.cpp
#../src/keyframe.cpp
#../src/mapper.cpp
#../src/tracker.cpp
#../src/bundleadj.cpp
#../src/initializer.cpp
#../src/wavelet.cpp
#)

##eval orb initializer
#add_executable(eval_orb_initializer eval_orb_initializer.cpp
#../src/camera.cpp
#../src/image.cpp
#../src/camera.cpp
##../src/renderer.cpp
#../src/environment.cpp
#../src/dtam.cpp
#../src/utils.cpp
#../src/epline.cpp
#../src/keyframe.cpp
#../src/mapper.cpp
#../src/tracker.cpp
#../src/bundleadj.cpp
#../src/initializer.cpp
#../src/wavelet.cpp
#)

##test tracking
#add_executable(test_tracking test_tracking.cpp
#../src/camera.cpp
#../src/image.cpp
#../src/camera.cpp
##../src/renderer.cpp
#../src/environment.cpp
#../src/dtam.cpp
#../src/utils.cpp
#../src/epline.cpp
#../src/keyframe.cpp
#../src/mapper.cpp
#../src/tracker.cpp
#../src/bundleadj.cpp
#../src/initializer.cpp
#../src/wavelet.cpp
#)

#test dso
add_executable(test_dso test_dso.cpp
../src/image.cpp
../src/CamerasContainer.cpp
../src/CameraForMapping.cpp
../src/CamCouple.cpp
../src/CandidatesActivator.cpp
../src/CoarseRegions.cpp
../src/dso.cpp
../src/camera.cpp
../src/Pyramid.cpp
#../src/spectator.cpp
../src/environment.cpp
../src/Tracker.cpp
#../src/Tree.cpp
#../src/dtam.cpp
../src/utils.cpp
../src/epline.cpp
../src/KeyframeHandler.cpp
../src/PointsContainer.cpp
../src/LinSystem.cpp
../src/PointsHandler.cpp
../src/spectator.cpp
#../src/mapper.cpp
#../src/tracker.cpp
#../src/bundleadj.cpp
../src/initializer.cpp
#../src/wavelet.cpp
)
target_link_libraries(test_dso ${PYTHON_LIBRARIES})
