/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_C16_DECODER_H
#define LSLIDAR_C16_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int8.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <lslidar_c16_msgs/msg/lslidar_c16_packet.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_point.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_scan.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_sweep.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_layer.hpp>

namespace lslidar_c16_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX        = 130.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.0025; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines forLSc16 support **/
static const int     FIRINGS_PER_BLOCK = 2;
static const int     SCANS_PER_FIRING  = 16;
static const double  BLOCK_TDURATION   = 110.592; // [µs]
static const double  DSR_TOFFSET       = 1;   
static const double  FIRING_TOFFSET    = 16;  

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double cos_scan_altitude[16] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZITR {
  PCL_ADD_POINT4D
  uint8_t intensity;
  double timestamp;
  uint32_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

class LslidarC16Decoder {
public:

    LslidarC16Decoder(std::shared_ptr<rclcpp::Node> node);
    LslidarC16Decoder(const LslidarC16Decoder&) = delete;
    LslidarC16Decoder operator=(const LslidarC16Decoder&) = delete;
    ~LslidarC16Decoder() {return;}

    bool initialize();

private:

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

    struct Firing {
        // Azimuth associated with the first shot within this firing.
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    bool checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void layerCallback(const std_msgs::msg::Int8& msg);
    void packetCallback(const lslidar_c16_msgs::msg::LslidarC16Packet& msg);
    // Publish data
    void publishPointCloud();
    void publishChannelScan();
    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        // azimuth = raw_azimuth / 100.0;
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // calc the means_point
    point_struct getMeans(std::vector<point_struct> clusters);

    // configuration degree base
    int point_num;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double angle3_disable_min;
    double angle3_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool use_gps_ts;
    bool publish_scan;
    bool apollo_interface;
    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    unsigned long sweep_start_time_nsec;
    uint32_t packet_start_time;
    int layer_num;
    Firing firings[FIRINGS_PER_PACKET];

    // ROS related parameters
    std::shared_ptr<rclcpp::Node> node;

    //std::string fixed_frame_id;
    std::string frame_id;

    lslidar_c16_msgs::msg::LslidarC16Sweep sweep_data;
    lslidar_c16_msgs::msg::LslidarC16Layer multi_scan;
    sensor_msgs::msg::PointCloud2 point_cloud_data;

    rclcpp::Subscription<lslidar_c16_msgs::msg::LslidarC16Packet>::SharedPtr packet_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr layer_sub;
    rclcpp::Publisher<lslidar_c16_msgs::msg::LslidarC16Sweep>::SharedPtr sweep_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<lslidar_c16_msgs::msg::LslidarC16Layer>::SharedPtr channel_scan_pub;

};

typedef PointXYZITR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lslidar_c16_decoder


POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_c16_decoder::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp)(uint32_t, ring, ring))
#endif
