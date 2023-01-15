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

#include <rclcpp/rclcpp.hpp>
#include <lslidar_c16_driver/lslidar_c16_driver.h>

class LSLidarC16DriverNode : public rclcpp::Node
{
public:
    LSLidarC16DriverNode():
            Node("lslidar_c16_driver_node")
    {
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<LSLidarC16DriverNode>();

    // start the driver
    RCLCPP_INFO(node->get_logger(), "namespace is %s", node->get_namespace());
    lslidar_c16_driver::LslidarC16Driver driver(node);
  if (!driver.initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Cannot initialize lslidar driver...");
    return 0;
  }
    // loop until shut down or end of file
    while(rclcpp::ok() && driver.polling()) {
        rclcpp::spin_some(node);
    }

    return 0;
}
