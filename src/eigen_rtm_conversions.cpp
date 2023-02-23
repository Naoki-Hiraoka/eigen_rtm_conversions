#include <eigen_rtm_conversions/eigen_rtm_conversions.h>

namespace eigen_rtm_conversions{
  void vectorEigenToRTM(const Eigen::Vector3d& e, RTC::Vector3D& r){
    r.x = e[0];
    r.y = e[1];
    r.z = e[2];
  }
  void vectorRTMToEigen(const RTC::Vector3D& r, Eigen::Vector3d& e){
    e[0] = r.x;
    e[1] = r.y;
    e[2] = r.z;
  }
  void pointEigenToRTM(const Eigen::Vector3d& e, RTC::Point3D& r){
    r.x = e[0];
    r.y = e[1];
    r.z = e[2];
  }
  void pointRTMToEigen(const RTC::Point3D& r, Eigen::Vector3d& e){
    e[0] = r.x;
    e[1] = r.y;
    e[2] = r.z;
  }
  void pointEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>::TranslationPart& e, RTC::Point3D& r){
    r.x = e[0];
    r.y = e[1];
    r.z = e[2];
  }
  void pointRTMToEigen(const RTC::Point3D& r, Eigen::Ref<Eigen::Transform<double, 3, Eigen::AffineCompact>::TranslationPart> e){
    e[0] = r.x;
    e[1] = r.y;
    e[2] = r.z;
  }
  void orientationEigenToRTM(const Eigen::Matrix3d& e, RTC::Orientation3D& r){
    Eigen::Vector3d ypr=e.eulerAngles(2,1,0);
    r.r=ypr[2];
    r.p=ypr[1];
    r.y=ypr[0];
  }
  void orientationRTMToEigen(const RTC::Orientation3D& r, Eigen::Matrix3d& e){
    e = (Eigen::AngleAxisd(r.y, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(r.p, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(r.r, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  void orientationEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>::LinearPart& e, RTC::Orientation3D& r){
    Eigen::Vector3d ypr=e.eulerAngles(2,1,0);
    r.r=ypr[2];
    r.p=ypr[1];
    r.y=ypr[0];
  }
  void orientationRTMToEigen(const RTC::Orientation3D& r, Eigen::Ref<Eigen::Transform<double, 3, Eigen::AffineCompact>::LinearPart> e){
    e = (Eigen::AngleAxisd(r.y, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(r.p, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(r.r, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  void poseEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>& e, RTC::Pose3D& r){
    pointEigenToRTM(e.translation(),r.position);
    orientationEigenToRTM(e.linear(),r.orientation);
  }
  void poseRTMToEigen(const RTC::Pose3D& r, Eigen::Transform<double, 3, Eigen::AffineCompact>& e){
    pointRTMToEigen(r.position,e.translation());
    orientationRTMToEigen(r.orientation,e.linear());
  }
};
