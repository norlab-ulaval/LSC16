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

#ifndef LSLIDAR_C16_DRIVER_H
#define LSLIDAR_C16_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <lslidar_c16_msgs/msg/lslidar_c16_packet.hpp>
#include <lslidar_c16_msgs/msg/lslidar_c16_scan_unified.hpp>

namespace lslidar_c16_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarC16Driver {
public:

    LslidarC16Driver(std::shared_ptr<rclcpp::Node> node);
    ~LslidarC16Driver();

    bool initialize();
    bool polling();

    void initTimeStamp(void);
    void getFPGA_GPSTimeStamp(lslidar_c16_msgs::msg::LslidarC16Packet& packet);

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_c16_msgs::msg::LslidarC16Packet& msg);

    // Ethernet relate variables
    std::string lidar_ip_string;
    std::string group_ip_string;
    in_addr lidar_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
    int cnt_gps_ts;
    bool use_gps_;
	bool add_multicast;
    // ROS related variables
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<lslidar_c16_msgs::msg::LslidarC16Packet>::SharedPtr packet_pub;

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    uint64_t pointcloudTimeStamp;
    uint64_t GPSStableTS;
    uint64_t GPSCountingTS;
    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
};

} // namespace lslidar_driver

#endif // _LSLIDAR_C16_DRIVER_H_
