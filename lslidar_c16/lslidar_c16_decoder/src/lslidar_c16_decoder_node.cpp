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

#include <lslidar_c16_decoder/lslidar_c16_decoder.h>

class LSLidarC16DecoderNode : public rclcpp::Node
{
public:
    LSLidarC16DecoderNode():
            Node("lslidar_c16_decoder_node")
    {
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<LSLidarC16DecoderNode>();

    lslidar_c16_decoder::LslidarC16Decoder decoder(node);

    if (!decoder.initialize()) {
        RCLCPP_INFO(node->get_logger(), "Cannot initialize the decoder...");
        return -1;
    }

    rclcpp::spin(node);

    return 0;
}
