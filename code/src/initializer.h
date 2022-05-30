#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dso; //forward declaration


class Initializer{

  public:
    Initializer(Dso* dso):
    dso_(dso)
    {};

    void compute_cv_K();
    void extractCorners();
    // void showCornersRef();
    void trackCornersLK();
    float getOpticalFlowDist();
    // void showCornersTrackSequence();
    void showCornersTrackCurr();
    void showCornersTrackRef();
    bool findPose();
    // void corners2activePoints();

  protected:

    Dso* dso_;
    //
    CameraForMapping* ref_frame_;
    int ref_frame_idx_;
    std::vector<cv::Point2f> corners_vec_ref;
    std::vector<cv::Point2f> corners_vec_curr;
    std::vector<float> errors_vec;
    std::vector<uchar> status_vec;
    std::vector<uchar> inliers_vec;

    cv::Mat cv_K;

    std::vector<colorRGB> colors;

    // const Image<float>* getReferenceImage();
    // const Image<float>* getCurrentImage();
    // const Image<float>* getPrevImage();

    cv::Mat findEssentialMatrix();
    // cv::Mat findFundamentalMatrix();
    // cv::Mat findHomography();
    // cv::Mat fundamental2Essential(cv::Mat& F);
    Eigen::Isometry3f essential2pose(cv::Mat& E);
    // Eigen::Isometry3f homography2pose(cv::Mat& H);
    Eigen::Isometry3f computeRelativePoseGt();

    void initializeColors();

};
