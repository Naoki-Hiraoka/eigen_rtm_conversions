#ifndef EIGEN_RTM_CONVERSIONS_H
#define EIGEN_RTM_CONVERSIONS_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <Eigen/Eigen>

namespace eigen_rtm_conversions{
  template<int rows>
  void vectorEigenToRTM(const Eigen::Matrix<double,rows,1>& e, _CORBA_Sequence<double>& r){
    r.length(e.size());
    for(int i=0;i<e.size();i++) r[i] = e[i];
  }
  template<int rows>
  void vectorRTMToEigen(const _CORBA_Sequence<double>& r, Eigen::Matrix<double,rows,1>& e){
    e.resize(r.length());
    for(int i=0;i<r.length();i++) e[i] = r[i];
  }
  void vectorEigenToRTM(const Eigen::Vector3d& e, RTC::Vector3D& r);
  void vectorRTMToEigen(const RTC::Vector3D& r, Eigen::Vector3d& e);
  void pointEigenToRTM(const Eigen::Vector3d& e, RTC::Point3D& r);
  void pointRTMToEigen(const RTC::Point3D& r, Eigen::Vector3d& e);
  void pointEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>::TranslationPart& e, RTC::Point3D& r);
  void pointRTMToEigen(const RTC::Point3D& r, Eigen::Ref<Eigen::Transform<double, 3, Eigen::AffineCompact>::TranslationPart> e);
  void orientationEigenToRTM(const Eigen::Matrix3d& e, RTC::Orientation3D& r);
  void orientationRTMToEigen(const RTC::Orientation3D& r, Eigen::Matrix3d& e);
  void orientationEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>::LinearPart& e, RTC::Orientation3D& r);
  void orientationRTMToEigen(const RTC::Orientation3D& r, Eigen::Ref<Eigen::Transform<double, 3, Eigen::AffineCompact>::LinearPart> e);
  void poseEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>& e, RTC::Pose3D& r);
  void poseRTMToEigen(const RTC::Pose3D& r, Eigen::Transform<double, 3, Eigen::AffineCompact>& e);
};

#endif

