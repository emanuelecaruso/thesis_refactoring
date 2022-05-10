#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <unistd.h>
#include "opencv2/opencv.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <mutex>



//ds opencv keys
#define OPENCV_KEY_UP 2490368
#define OPENCV_KEY_DOWN 2621440
#define OPENCV_KEY_LEFT 2424832
#define OPENCV_KEY_RIGHT 2555904
#define OPENCV_KEY_SPACE 32
#define OPENCV_KEY_DELETE 3014656
#define OPENCV_KEY_ESCAPE 27

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;
typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;


typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;
typedef Eigen::Matrix<float, 9, 9> Matrix9f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;

typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;
typedef Eigen::Matrix<float, 6, 2> Matrix6_2f;
typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;

typedef cv::Vec3f colorRGB;


template <class T>
bool isNan(const T& m){
  for (int i=0; i< m.rows(); i++) {
    for (int j=0; j< m.cols(); j++) {
float v = m(i,j);
if ( std::isnan( v ) )
  return true;
    }
  }
  return false;
}


// inline Eigen::Isometry3f v2t(const Vector6f& t){
//   Eigen::Isometry3f T;
//   T.setIdentity();
//   T.translation()=t.head<3>();
//   float w=t.block<3,1>(3,0).squaredNorm();
//   if (w<1) {
//     w=sqrt(1-w);
//     T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
//   } else {
//     T.linear().setIdentity();
//   }
//   return T;
// }
//
// inline Vector6f t2v(const Eigen::Isometry3f& t){
//   Vector6f v;
//   v.head<3>()=t.translation();
//   Eigen::Quaternionf q(t.linear());
//   v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
//   if (q.w()<0)
//     v.block<3,1>(3,0) *= -1.0f;
//   return v;
// }
//
// inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
//   Eigen::Isometry2f T;
//   T.setIdentity();
//   T.translation()=t.head<2>();
//   float c = cos(t(2));
//   float s = sin(t(2));
//   T.linear() << c, -s, s, c;
//   return T;
// }
//
// inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
//   Eigen::Vector3f v;
//   v.head<2>()=t.translation();
//   v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
//   return v;
// }

inline Eigen::Matrix3f Rx(float rot_x){
  float c=cos(rot_x);
  float s=sin(rot_x);
  Eigen::Matrix3f R;
  R << 1,  0, 0,
    0,  c,  -s,
    0,  s,  c;
  return R;
}

inline Eigen::Matrix3f Ry(float rot_y){
  float c=cos(rot_y);
  float s=sin(rot_y);
  Eigen::Matrix3f R;
  R << c,  0,  s,
    0 , 1,  0,
    -s,  0, c;
  return R;
}

inline Eigen::Matrix3f Rz(float rot_z){
  float c=cos(rot_z);
  float s=sin(rot_z);
  Eigen::Matrix3f R;
  R << c,  -s,  0,
    s,  c,  0,
    0,  0,  1;
  return R;
}


// inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
//   Eigen::Isometry3f T;
//   T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
//   T.translation()=v.head<3>();
//   return T;
// }


inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
  Eigen::Matrix3f S;
  S << 0, -v[2], v[1],
    v[2], 0, -v[0],
    -v[1], v[0], 0;
  return S;
}




/** \typedef UnsignedCharImage
 * \brief An unsigned char cv::Mat.
 */
typedef cv::Mat_<unsigned char> UnsignedCharImage;

/** \typedef CharImage
 * \brief A char cv::Mat.
 */
typedef cv::Mat_<char> CharImage;

/** \typedef UnsignedShortImage
 * \brief An unsigned short cv::Mat.
 */
typedef cv::Mat_<unsigned short> UnsignedShortImage;

/** \typedef UnsignedIntImage
 * \brief An unsigned int cv::Mat.
 */
typedef cv::Mat_<unsigned int> UnsignedIntImage;

/** \typedef IntImage
 * \brief An int cv::Mat.
 */
typedef cv::Mat_<int> IntImage;

/** \typedef FloatImage
 * \brief A float cv::Mat.
 */
typedef cv::Mat_<float> FloatImage;

/** \typedef Float3Image
 * \brief A float cv::Mat.
 */
typedef cv::Mat_<cv::Vec3f> Float3Image;

/** \typedef DoubleImage
 * \brief A double cv::Mat.
 */
typedef cv::Mat_<double> DoubleImage;

/** \typedef RawDepthImage
 * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
 */
typedef UnsignedShortImage RawDepthImage;

/** \typedef IndexImage
 * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
 */
typedef IntImage IndexImage;

/** \typedef DepthImage
 * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
 */
typedef cv::Mat_< cv::Vec3b > RGBImage;

typedef cv::Mat_< uchar > GreyImage;

typedef std::vector< cv::Vec3b > RGBVector;

typedef std::vector<int> IntVector;

typedef std::vector<float> FloatVector;

typedef std::pair<int,int> IntPair;

typedef std::vector<IntPair > IntPairVector;



// inline float extractRollAngle(Eigen::Isometry3f& T){
//   Eigen::Matrix3f R=T.linear();
//   float r10 = R(1,0);
//   float r00 = R(0,0);
//   return atan2(r10,r00);
// }

#define PI 3.14159265
#define EPS 0.00000000001

struct CamParameters{
  const int resolution_x;
  const int resolution_y;
  const float aspect;
  const float width;
  const float height;
  const float lens;
  const float min_depth;
  const float max_depth;
  const float pixel_width;
  const float pixel_meter_ratio;

  CamParameters(int resolution_x_, int resolution_y_,
   float width_,float lens_,float min_depth_,float max_depth_):
  resolution_x(resolution_x_), resolution_y(resolution_y_), aspect((float)resolution_x_/(float)resolution_y_),
  width(width_), height(width_/aspect), lens(lens_), min_depth(min_depth_), max_depth(max_depth_),
  pixel_width(width/(float)resolution_x), pixel_meter_ratio((float)resolution_x/width)
  { };

  // clone
  CamParameters(const CamParameters* cam_parameters):
  resolution_x(cam_parameters->resolution_x), resolution_y(cam_parameters->resolution_y), aspect((float)resolution_x/(float)resolution_y),
  width(cam_parameters->width), height(width/aspect), lens(cam_parameters->lens), min_depth(cam_parameters->min_depth), max_depth(cam_parameters->max_depth),
  pixel_width(width/(float)resolution_x), pixel_meter_ratio((float)resolution_x/width)
  { };

  inline void printMembers() const {
    std::cout << "lens: " << lens << std::endl;
    std::cout << "aspect: " << aspect << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;
    std::cout << "resolution_x: " << resolution_x << std::endl;
    std::cout << "resolution_y: " << resolution_y << std::endl;
    std::cout << "max_depth: " << max_depth << std::endl;
    std::cout << "min_depth: " << min_depth << std::endl;
  }


  inline colorRGB invdepthToRgb( const float invdepth) const {

    float min_depth_ = min_depth;
    float invdepth_normalized = min_depth_*invdepth;

    float H = 230*(1-invdepth_normalized);

    float s = 1;
    float v = 0.7;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    int R = (r+m);
    int G = (g+m);
    int B = (b+m);

    colorRGB color = colorRGB(b,g,r);

    return color;
  }

};


static bool debug_mode = 1;
static std::mutex mu_cout;
static std::mutex mu_show_img;
static std::mutex mu_waitkey;

inline void waitkey(int num){
  std::lock_guard<std::mutex> locker(mu_waitkey);
  cv::waitKey(num);
}

const float colorRGB_maxval = 1;
const int colorRGB_CODE = CV_32FC3;


typedef float pixelIntensity;
const float pixelIntensity_maxval = 1;
const int pixelIntensity_CODE = CV_32FC1;

typedef Eigen::Vector2f pxl;

struct Cp // Colored point (in 3D)
{
  Eigen::Vector3f point;
  cv::Vec3b color;
};



inline void sharedCout(const std::string& msg){
  std::lock_guard<std::mutex> locker(mu_cout);
  std::cout << msg << std::endl;
}

inline void sharedCoutDebug(const std::string& msg){
  std::lock_guard<std::mutex> locker(mu_cout);
  if (debug_mode){
    std::cout << msg << std::endl;
  }
}

inline float l1Norm(cv::Vec3f vec){
  return abs(vec[0])+abs(vec[1])+abs(vec[2]);
}


const colorRGB black = colorRGB(0,0,0);
const colorRGB white = colorRGB(colorRGB_maxval,colorRGB_maxval,colorRGB_maxval);
const colorRGB grey = colorRGB(colorRGB_maxval/2,colorRGB_maxval/2,colorRGB_maxval/2);
const colorRGB magenta = colorRGB(colorRGB_maxval,0,colorRGB_maxval);
const colorRGB orange = colorRGB(0,colorRGB_maxval*0.546875,colorRGB_maxval);
const colorRGB red = colorRGB(0,0,colorRGB_maxval);
const colorRGB green = colorRGB(0,colorRGB_maxval,0);
const colorRGB blue = colorRGB(colorRGB_maxval,0,0);
const colorRGB cyan = colorRGB(colorRGB_maxval,colorRGB_maxval,0);
const colorRGB yellow = colorRGB(0,colorRGB_maxval,colorRGB_maxval);


inline float radiansSub(float rad1, float rad2){

  if(rad2<0 || rad2>2*PI){
    std::cout << "OAOOO " << rad2 << std::endl;
  }
  assert(rad1>=0 && rad1<=2*PI);
  assert(rad2>=0 && rad2<=2*PI);

  float sub = rad1-rad2;
  int i=0;
  while(true){
    if (sub<-PI-EPS){
      sub+=2*PI;
    }else if(sub>PI+EPS){
      sub-=2*PI;
    }else break;
    i++;
    if(i>1){
      std::cout << "WHYYYYYYYYYYYYYYYY " << rad1 << " " << rad2 <<  std::endl;
      exit(1);
      // break;
    }

  }
  return sub;
}

inline float squareNorm(float a){
  return pow(a,2);
}

inline float squareNormDerivative(float a){
  return 2*a;
}

inline float huberNorm(float a, float b){
  float huber_norm;
  if (abs(a)<=b){
    huber_norm= (pow(a,2))/(2*b);
  }else{
    huber_norm= (abs(a)-b/2);
  }
  return huber_norm;
}

inline float huberNormWithOmega(float a, float b, float omega){
  float huber_norm;
  if ((abs(a)*sqrt(omega))<=b){
    huber_norm= (pow(a,2)*omega)/(2*b);
  }else{
    huber_norm= ((abs(a)*abs(sqrt(omega)))-b/2);
  }
  return huber_norm;
}

inline float huberNormDerivative(float a, float b){
  float huber_norm_der;
  if (abs(a)<=b){
    huber_norm_der= a/b;
  }else if (a>0){
    huber_norm_der= 1;
  }else if (a<0){
    huber_norm_der= -1;
  }
  return huber_norm_der;
}


inline Vector6f t2v(const Eigen::Isometry3f& T){
  // t_inv is new_T_old
  Vector6f t;
  t.head<3>()=T.translation();
  Eigen::Matrix3f R = T.linear();
  // Eigen::Vector3f ea = T.linear().eulerAngles(0, 1, 2); //XYZ convention
  // Eigen::Vector3f ea = T.linear().eulerAngles(2, 1, 0); //XYZ convention
  // t.tail<3>()=ea;
  t[3]=std::atan2(-R(1,2) , R(2,2));
  t[4]=std::atan2(R(0,2) , (sqrt(1-R(0,2)*R(0,2))));
  t[5]=std::atan2(-R(0,1) , R(0,0));
  return t;
}

inline Eigen::Isometry3f v2t(const Vector6f& t){
  // t_inv is new_T_old
  Eigen::Isometry3f T;
  T.translation()=t.head<3>();
  float a = t(3);
  float b = t(4);
  float c = t(5);
  T.linear() <<                         cos(b)*cos(c),                       -cos(b)*sin(c),         sin(b),
                 cos(a)*sin(c) + cos(c)*sin(a)*sin(b), cos(a)*cos(c) - sin(a)*sin(b)*sin(c), -cos(b)*sin(a),
                 sin(a)*sin(c) - cos(a)*cos(c)*sin(b), cos(c)*sin(a) + cos(a)*sin(b)*sin(c),  cos(a)*cos(b);

  return T;
}

inline Eigen::Isometry3f v2t_inv(Vector6f& t){
  // t_inv is old_T_new
  Eigen::Isometry3f T;

  float a = t(3);
  float b = t(4);
  float c = t(5);
  T.linear() <<   cos(b)*cos(c), cos(a)*sin(c) + cos(c)*sin(a)*sin(b), sin(a)*sin(c) - cos(a)*cos(c)*sin(b),
                 -cos(b)*sin(c), cos(a)*cos(c) - sin(a)*sin(b)*sin(c), cos(c)*sin(a) + cos(a)*sin(b)*sin(c),
                         sin(b),                       -cos(b)*sin(a),                        cos(a)*cos(b);

  T.translation().x() = - t(1)*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) - t(2)*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) - t(0)*cos(b)*cos(c);
  T.translation().y() = t(0)*cos(b)*sin(c) - t(2)*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) - t(1)*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c));
  T.translation().z() = t(1)*cos(b)*sin(a) - t(2)*cos(a)*cos(b) - t(0)*sin(b);


  return T;
}

#define POSE_CONSTANT 0
#define VELOCITY_CONSTANT 1
#define PERS_GUESS 2

#define HUBER 0
#define QUADRATIC 1
#define LINEAR 2
#define TEST_ALL 0
#define TEST_ONLY_POSES 1
#define TEST_ONLY_POINTS 2
#define TEST_ONLY_POSES_ONLY_M 3
#define INTENSITY_ID 0
#define GRADIENT_ID 1
#define PHASE_ID 2

#define CAND_MODE 0
#define ACT_MODE 1

#define ALL_KFS_ON_LAST 0
#define ALL_KFS_ON_ALL_KFS 1
#define KF_ON_ALL_KFS 2




inline int lowerBound(const std::vector<int>& vec, int value) {
  auto const it = std::lower_bound(vec.begin(), vec.end(), value);
  if (it == vec.end()) { return -1; }

  return *it;
}

inline int upperBound(const std::vector<int>& vec, int value) {
  auto const it = std::upper_bound(vec.begin(), vec.end(), value);
  if (it == vec.end()) { return -1; }

  return *it;
}

template <class T>
inline T pinvDense(const T& matrix){
  return matrix.completeOrthogonalDecomposition().pseudoInverse();
}

inline float getPinvThreshold(const Eigen::VectorXf& singular_values){
  // float sv_sum = 0;
  float max = 0;
  float min = FLT_MAX;
  int idx_max = -1;
  int idx_min = -1;
  int size = singular_values.size();
  float sv_sum = 0;
  for(int i=0; i<size; i++){
    sv_sum += singular_values[i];
    if(singular_values[i]>max){
      max=singular_values[i];
      idx_max=i;
    }
    if(singular_values[i]<min){
      min=singular_values[i];
      idx_min=i;
    }
  }
  float sv_average = sv_sum/size;

  // float thresh = 2.858*sv_average;
  // float thresh = sv_average;
  // float thresh = 0.5*sv_average;
  // float thresh = 50;
  // float thresh = 2.309 *size*(sv_average/1000);
  float thresh = max*7.15256e-07*size;
  // float thresh = 1000000;

  // std::cout << "thresh " << thresh << ", avg: " << sv_average << ", max " << max << " i " << idx_max << ", min " << min << " i " << idx_min << std::endl;

  // thresh = 0.01;
  return thresh;
}


inline Eigen::DiagonalMatrix<float,Eigen::Dynamic> invDiagonalMatrix(Eigen::VectorXf& vec_in){

  Eigen::DiagonalMatrix<float,Eigen::Dynamic> mat_out;
  int size = vec_in.size();
  mat_out.resize(size);

  for(int i=0; i<size; i++){
    float val = vec_in[i];
      mat_out.diagonal()[i]=(1.0/val);
  }

  return mat_out;
}

inline Eigen::DiagonalMatrix<float,Eigen::Dynamic> pinvDiagonalMatrix(Eigen::DiagonalMatrix<float,Eigen::Dynamic>& mat_in){

  Eigen::DiagonalMatrix<float,Eigen::Dynamic> mat_out;
  int size = mat_in.diagonal().size();
  float thresh = getPinvThreshold(mat_in.diagonal());
  mat_out.resize(size);
  int count = 0;
  for(int i=0; i<size; i++){
    float val = mat_in.diagonal()[i];
    if(val>thresh)
    // if(val!=0)
      mat_out.diagonal()[i]=(1.0/val);
    else{
      mat_out.diagonal()[i]=0;
      count++;
    }
  }
  std::cout << "discarded: " << count << " out of " << size << std::endl;

  return mat_out;
}

template<class T>
inline T* pinv(T& A){
  //SVD decomposition
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A);
  Eigen::VectorXf singular_values = svd.singularValues();
  Eigen::MatrixXf U = svd.matrixU();
  Eigen::MatrixXf V = svd.matrixV();
  Eigen::MatrixXf U_transp = U.transpose();
  int m = U.rows();
  int n = V.rows();
  assert(singular_values.size()==m);

  float threshold = getPinvThreshold(singular_values);

  // std::cout << "thresh " << threshold << std::endl;


  // hard thresholding on singular values
  for(int i=0; i<singular_values.size(); i++){
    float val = singular_values[i];
    if(val>threshold)
      singular_values[i]=1.0/val;
    else
      singular_values[i]=0;
  }

  Eigen::MatrixXf EPS_pinv = Eigen::MatrixXf::Zero(n, m);
  for(int i=0; i<singular_values.size(); i++){
    EPS_pinv(i,i)=singular_values[i];
  }

  T* A_pinv = new T(V*EPS_pinv*U_transp);

  // std::cout << V << "\n\n" << EPS_pinv << "\n\n" << U_transp << std::endl;

  return A_pinv;
}

inline float rotation2angle(const Eigen::Matrix3f& rot_mat){
  assert(rot_mat.allFinite());

  Eigen::AngleAxisf angle_axis(rot_mat);

  return angle_axis.angle();
}

inline float randZeroMeanNoise(float range){
  float out = range/2 - ((float)rand()/RAND_MAX) * range;
  return out;
}

inline float getEuclideanDistance(Eigen::Vector2f& uv1, Eigen::Vector2f& uv2){
  return (uv2-uv1).norm();
}


inline pxl cvpoint2pxl(cv::Point2i& point ){
  pxl pixel;
  pixel.x()=(float)(point.x)+0.5; // point.y is col
  pixel.y()=(float)(point.y)+0.5; // point.x is row
  return pixel;
}

inline pxl cvpoint2pxl(cv::Point2f& point ){
  pxl pixel;
  pixel.x()=point.x; // point.y is col
  pixel.y()=point.y; // point.x is row
  return pixel;
}

template <class T>
inline void removeFromVecByIdx(std::vector<T>& v, int idx){
  assert(idx>=0 && idx<v.size());
  v.erase( v.begin()+idx );

}

template <class T>
inline void removeFromVecByElement(std::vector<T>& v, T element){
  int v_size = v.size();
  v.erase(std::remove(v.begin(), v.end(), element), v.end());
  assert(v_size==v.size()+1);
}

template <class T>
inline int getIndex(std::vector<T> v, T element)
{
    auto it = std::find(v.begin(), v.end(), element);

    assert(it!=v.end());

    // calculating the index
    int index = it - v.begin();

    return index;

}

inline void removeRowCol(Eigen::MatrixXf& matrix, unsigned int rowcolToRemove)
{
  unsigned int numRows = matrix.rows()-1;
  unsigned int numCols = matrix.cols()-1;

  if( rowcolToRemove < numRows ){
    matrix.block(rowcolToRemove,0,numRows-rowcolToRemove,numCols) = matrix.block(rowcolToRemove+1,0,numRows-rowcolToRemove,numCols);
    matrix.block(0,rowcolToRemove,numRows,numCols-rowcolToRemove) = matrix.block(0,rowcolToRemove+1,numRows,numCols-rowcolToRemove);
  }

  matrix.conservativeResize(numRows,numCols);
}

inline void removeElement(Eigen::VectorXf& vec, unsigned int idx)
{
  unsigned int size = vec.size()-1;

  if( idx < size ){
    vec.segment(idx,size-idx) = vec.segment(idx+1,size-idx);
  }

  vec.conservativeResize(size);
}



inline void appendRowCol(Eigen::MatrixXf& matrix)
{
  unsigned int numRows = matrix.rows()+1;
  unsigned int numCols = matrix.cols()+1;

  matrix.conservativeResize(numRows,numCols);

  matrix.col(matrix.cols()-1).setZero();
  matrix.row(matrix.rows()-1).setZero();
}

inline void appendElement(Eigen::VectorXf& vec)
{
  unsigned int size = vec.size()+1;

  vec.conservativeResize(size);

  vec(vec.size()-1)=0;

}
