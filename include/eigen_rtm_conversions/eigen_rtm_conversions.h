#ifndef EIGEN_RTM_CONVERSIONS_H
#define EIGEN_RTM_CONVERSIONS_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <Eigen/Eigen>

namespace eigen_rtm_conversions{
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
  void positionEigenToRTM(const Eigen::Transform<double, 3, Eigen::AffineCompact>& e, RTC::Pose3D& r);
  void positionRTMToEigen(const RTC::Pose3D& r, Eigen::Transform<double, 3, Eigen::AffineCompact>& e);
};

#endif

