link_libraries(${OpenCV_LIBS})


#test c1 c2
add_executable(test_tracking test_tracking.cpp
../src/image.cpp
../src/CamerasContainer.cpp
../src/CameraForMapping.cpp
../src/CamCouple.cpp
../src/CandidatesActivator.cpp
../src/dso.cpp
../src/camera.cpp
../src/Pyramid.cpp
../src/environment.cpp
../src/Tracker.cpp
../src/utils.cpp
../src/epline.cpp
../src/parameters.cpp
../src/KeyframeHandler.cpp
../src/PointsContainer.cpp
../src/LinSystemTracking.cpp
../src/LinSystemBA.cpp
../src/LinSystem.cpp
../src/BundleAdj.cpp
../src/PointsHandler.cpp
../src/spectator.cpp
../src/initializer.cpp
)
target_link_libraries(test_tracking ${PYTHON_LIBRARIES})

#test c1 c2
add_executable(test_coeffs test_coeffs.cpp
../src/image.cpp
../src/CamerasContainer.cpp
../src/CameraForMapping.cpp
../src/CamCouple.cpp
../src/CandidatesActivator.cpp
../src/dso.cpp
../src/camera.cpp
../src/Pyramid.cpp
../src/environment.cpp
../src/Tracker.cpp
../src/utils.cpp
../src/epline.cpp
../src/parameters.cpp
../src/KeyframeHandler.cpp
../src/PointsContainer.cpp
../src/LinSystemTracking.cpp
../src/LinSystemBA.cpp
../src/LinSystem.cpp
../src/BundleAdj.cpp
../src/PointsHandler.cpp
../src/spectator.cpp
../src/initializer.cpp
)
target_link_libraries(test_coeffs ${PYTHON_LIBRARIES})

#test dso
add_executable(test_dso test_dso.cpp
../src/image.cpp
../src/CamerasContainer.cpp
../src/CameraForMapping.cpp
../src/CamCouple.cpp
../src/CandidatesActivator.cpp
../src/dso.cpp
../src/camera.cpp
../src/Pyramid.cpp
../src/environment.cpp
../src/Tracker.cpp
../src/utils.cpp
../src/epline.cpp
../src/parameters.cpp
../src/KeyframeHandler.cpp
../src/PointsContainer.cpp
../src/LinSystemTracking.cpp
../src/LinSystemBA.cpp
../src/LinSystem.cpp
../src/BundleAdj.cpp
../src/PointsHandler.cpp
../src/spectator.cpp
../src/initializer.cpp
)
target_link_libraries(test_dso ${PYTHON_LIBRARIES})
