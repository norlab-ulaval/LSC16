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

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <lslidar_c16_driver/lslidar_c16_driver.h>

namespace lslidar_c16_driver {

LslidarC16Driver::LslidarC16Driver(std::shared_ptr<rclcpp::Node> node):
    node(node),
    diagnostics(node),
    socket_id(-1){
    return;
}

LslidarC16Driver::~LslidarC16Driver() {
    (void) close(socket_id);
    return;
}

bool LslidarC16Driver::loadParameters() {

  //pnh.param("frame_id", frame_id, std::string("lslidar"));
  node->declare_parameter<std::string>("lidar_ip", std::string("192.168.1.222"));
  node->get_parameter("lidar_ip", lidar_ip_string);
  node->declare_parameter<int>("device_port", 2368);
  node->get_parameter("device_port", UDP_PORT_NUMBER);
  node->declare_parameter<bool>("add_multicast", false);
  node->get_parameter("add_multicast", add_multicast);
  node->declare_parameter<std::string>("group_ip", std::string("234.2.3.2"));
  node->get_parameter("group_ip", group_ip_string);
  inet_aton(lidar_ip_string.c_str(), &lidar_ip);
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening UDP socket: address " << lidar_ip_string);
  if(add_multicast) RCLCPP_INFO_STREAM(node->get_logger(), "Opening UDP socket: group_address " << group_ip_string);
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
  return true;
}

bool LslidarC16Driver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_C16");
  // c16 publishs 20*16 thousands points per second.
  // Each packet contains 12 blocks. And each block
  // contains 32 points. Together provides the
  // packet rate.
  const double diag_freq = 16*20000.0 / (12*32);
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  RCLCPP_INFO(node->get_logger(), "expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic.reset(new TopicDiagnostic(
                         "lslidar_packets", diagnostics,
                         FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
                         TimeStampStatusParam()));

    // Output
    packet_pub = node->create_publisher<lslidar_c16_msgs::msg::LslidarC16Packet>("lslidar_packet", 100);
    return true;
}

bool LslidarC16Driver::openUDPPort() {
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: RCLCPP_ERROR errno
        return false;
    }
    //add multicast
	if(add_multicast){
       ip_mreq groupcast;
       groupcast.imr_interface.s_addr=INADDR_ANY;
       groupcast.imr_multiaddr.s_addr=inet_addr(group_ip_string.c_str());
    
       if(setsockopt(socket_id,IPPROTO_IP,IP_ADD_MEMBERSHIP,(char*)&groupcast,sizeof(groupcast))<0) {
          perror("set multicast error");
          close(socket_id);
          return false;
       }
	}
    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block");
        return false;
    }

    return true;
}

bool LslidarC16Driver::initialize() {

    this->initTimeStamp();

    if (!loadParameters()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot create all ROS IO...");
        return false;
    }

    if (!openUDPPort()) {
        RCLCPP_ERROR(node->get_logger(), "Cannot open UDP port...");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Initialised lslidar c16 without error");
    return true;
}

int LslidarC16Driver::getPacket(lslidar_c16_msgs::msg::LslidarC16Packet& packet) {

    double time1 = node->get_clock()->now().seconds();

    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 2000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (rclcpp::ok())
    {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    RCLCPP_ERROR(node->get_logger(), "poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                RCLCPP_WARN(node->get_logger(), "lslidar poll() timeout");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                RCLCPP_ERROR(node->get_logger(), "poll() reports lslidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(socket_id, &packet.data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);

//        RCLCPP_DEBUG_STREAM(node->get_logger(), "incomplete lslidar packet read: "
//                         << nbytes << " bytes");

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                RCLCPP_INFO(node->get_logger(), "recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( lidar_ip_string != "" && sender_address.sin_addr.s_addr != lidar_ip.s_addr )
                continue;
            else
                break; //done
        }



    }
    this->getFPGA_GPSTimeStamp(packet);

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    double time2 = node->get_clock()->now().seconds();
//    packet->stamp = ros::Time((time2 + time1) / 2.0);
    //packet->stamp = this->timeStamp;
		packet.stamp = node->get_clock()->now();
    return 0;
}

bool LslidarC16Driver::polling()
{
    lslidar_c16_msgs::msg::LslidarC16Packet packet;

    // Since the lslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    //for (int i = 0; i < config_.npackets; ++i)
    //  {
    //    while (true)
    //      {
    //        // keep reading until full packet received
    //        int rc = input_->getPacket(&scan->packets[i]);
    //        if (rc == 0) break;       // got a full packet?
    //        if (rc < 0) return false; // end of file reached?
    //      }
    //  }
    while (true)
    {
        // keep reading until full packet received
        int rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }

    // publish message using time of last packet read
    RCLCPP_DEBUG(node->get_logger(), "Publishing a full lslidar scan.");
    packet_pub->publish(packet);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet.stamp);
    diagnostics.force_update();

    return true;
}

void LslidarC16Driver::initTimeStamp(void)
{
    int i;

    for(i = 0;i < 10;i ++)
    {
        this->packetTimeStamp[i] = 0;
    }
    this->pointcloudTimeStamp = 0;

    this->timeStamp = rclcpp::Time(0.0);
}

void LslidarC16Driver::getFPGA_GPSTimeStamp(lslidar_c16_msgs::msg::LslidarC16Packet& packet)
{
    unsigned char head2[] = {packet.data[0],packet.data[1],packet.data[2],packet.data[3]};

    if(head2[0] == 0xA5 && head2[1] == 0xFF)
    {
        if(head2[2] == 0x00 && head2[3] == 0x5A)
        {
            this->packetTimeStamp[4] = packet.data[41];
            this->packetTimeStamp[5] = packet.data[40];
            this->packetTimeStamp[6] = packet.data[39];
            this->packetTimeStamp[7] = packet.data[38];
            this->packetTimeStamp[8] = packet.data[37];
            this->packetTimeStamp[9] = packet.data[36];

            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8]-1;
            cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
            this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));

            if (GPSCountingTS != this->pointcloudTimeStamp)
            {
                cnt_gps_ts = 0;
                GPSCountingTS = this->pointcloudTimeStamp;
            }
            else if (cnt_gps_ts == 3)
            {
                GPSStableTS = GPSCountingTS;
            }
            else
            {
                cnt_gps_ts ++;
            }
//            RCLCPP_DEBUG(node->get_logger(), "GPS: y:%d m:%d d:%d h:%d m:%d s:%d",
//                      cur_time.tm_year,cur_time.tm_mon,cur_time.tm_mday,cur_time.tm_hour,cur_time.tm_min,cur_time.tm_sec);
        }
    }
    else if(head2[0] == 0xFF && head2[1] == 0xEE)
    {
        uint64_t packet_timestamp;
        packet_timestamp = (packet.data[1200]  +
                            packet.data[1201] * pow(2, 8) +
                            packet.data[1202] * pow(2, 16) +
                            packet.data[1203] * pow(2, 24)) * 1e3;


        if ((last_FPGA_ts - packet_timestamp) > 0)
        {
            GPS_ts = GPSStableTS;

           // RCLCPP_DEBUG(node->get_logger(), "This is step time, using new GPS ts %lu", GPS_ts);
        }

        last_FPGA_ts = packet_timestamp;
        // timeStamp = rclcpp::Time(this->pointcloudTimeStamp+total_us/10e5);

        timeStamp = rclcpp::Time(GPS_ts, packet_timestamp);
//        RCLCPP_DEBUG(node->get_logger(), "ROS TS: %f, GPS: y:%d m:%d d:%d h:%d m:%d s:%d; FPGA: us:%lu",
//                  timeStamp.toSec(), GPS_ts, packet_timestamp);

    }
}

} // namespace lslidar_driver
