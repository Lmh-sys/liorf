#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gnss/gps_util.h"

class GPSProcessor {
public:
    using Ptr = std::shared_ptr<GPSProcessor>;

    GPSProcessor() = default;

    GPSProcessor(const std::vector<double> &extrinT,
                 const std::vector<double> &extrinR,
                 const std::vector<double> &initXYZ,
                 int lon0 = 117)
    {
        Configure(extrinT, extrinR, initXYZ, lon0);
    }

    void Configure(const std::vector<double> &extrinT,
                   const std::vector<double> &extrinR,
                   const std::vector<double> &initXYZ,
                   int lon0 = 117)
    {
        lon0_ = lon0;
        extrinT_gnss_ = extrinT.size() == 3 ? extrinT : std::vector<double>(3, 0.0);
        extrinR_gnss_ = extrinR.size() == 9 ? extrinR
                                            : std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        initXYZ_ = initXYZ;

        Eigen::Vector3d gnss_T_wrt_IMU(extrinT_gnss_[0], extrinT_gnss_[1], extrinT_gnss_[2]);
        Eigen::Matrix3d gnss_R_wrt_IMU;
        gnss_R_wrt_IMU << extrinR_gnss_[0], extrinR_gnss_[1], extrinR_gnss_[2], extrinR_gnss_[3], extrinR_gnss_[4],
            extrinR_gnss_[5], extrinR_gnss_[6], extrinR_gnss_[7], extrinR_gnss_[8];
        SetExtrinsic(gnss_T_wrt_IMU, gnss_R_wrt_IMU);

        if (!initXYZ_.empty() && initXYZ_[0] != 0) {
            SetOriginXYZ(initXYZ_[0], initXYZ_[1], initXYZ_[2]);
            gnss_inited_ = true;
        }
    }

    double time = 0.0;
    double local_x = 0.0;
    double local_y = 0.0;
    double local_z = 0.0;

    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_z = 0.0;

    void InitOrigin(double lat, double lon, double alt)
    {
        GPSUtil::ConvertLonLatToXY(lon, lat, origin_x, origin_y, lon0_);
        origin_z = alt;
        gnss_inited_ = true;
        std::cout << "Origin set to: " << std::fixed << std::setprecision(6) << origin_x << ", " << origin_y << ", "
                  << origin_z << std::endl;
    }

    void UpdatePosition(double lat, double lon, double alt)
    {
        double x = 0.0;
        double y = 0.0;
        GPSUtil::ConvertLonLatToXY(lon, lat, x, y, lon0_);
        Eigen::Vector3d gnss_pos = gnss_T_imu_ * Eigen::Vector3d(x - origin_x, y - origin_y, alt - origin_z);
        local_x = gnss_pos.x();
        local_y = gnss_pos.y();
        local_z = gnss_pos.z();
    }

    void SetOriginXYZ(double x, double y, double z)
    {
        origin_x = x;
        origin_y = y;
        origin_z = z;
        gnss_inited_ = true;
    }

    void TestXYZ(double lat, double lon, double alt)
    {
        double x = 0.0;
        double y = 0.0;
        GPSUtil::ConvertLonLatToXY(lon, lat, x, y, lon0_);
        Eigen::Vector3d gnss_pos(x - origin_x, y - origin_y, alt - origin_z);
        std::cout << "gnss_pos: " << gnss_pos.transpose() << std::endl;
        gnss_pos = gnss_T_imu_ * gnss_pos;
        std::cout << "gnss_pos: " << gnss_pos.transpose() << std::endl;
    }

    bool &getGnssInitStatus() { return gnss_inited_; }

    void SetExtrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot)
    {
        gnss_T_imu_.translation() = transl;
        gnss_T_imu_.linear() = rot;
    }

    void SetExtrinsic(const Eigen::Vector3d &transl) { SetExtrinsic(transl, Eigen::Matrix3d::Identity()); }

    Eigen::Vector3d GetLocalPosition() const { return Eigen::Vector3d(local_x, local_y, local_z); }

private:
    bool gnss_inited_ = false;
    int lon0_{117};

    Eigen::Affine3d gnss_T_imu_ = Eigen::Affine3d::Identity();
    std::vector<double> extrinT_gnss_{3, 0.0};
    std::vector<double> extrinR_gnss_{9, 0.0};
    std::vector<double> initXYZ_;
};
