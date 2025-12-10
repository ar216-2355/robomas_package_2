#include <rclcpp/rclcpp.hpp>
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/can_frame.hpp"
#include "robomas_package_2/msg/esc.hpp"
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>
#include <stdexcept>
#include <filesystem>
#include <fstream>

struct MotorValue {
    uint8_t mode{0};
    int value{0};
};

class SenderNode : public rclcpp::Node {
public:
    SenderNode(const std::string& ifname = "can0") : Node("Sender_node"), ifname_(ifname) {
        
        // kp, kd, ki, out_max, i_sum_max
        this->declare_parameter<std::vector<double>>("speed_gains_1", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_2", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_3", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_4", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_5", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_6", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_7", {17.0, 200.0, 0.05, 20000, 1000.0});
        this->declare_parameter<std::vector<double>>("speed_gains_8", {17.0, 200.0, 0.05, 20000, 1000.0});

        this->declare_parameter<std::vector<double>>("position_gains_1", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_2", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_3", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_4", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_5", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_6", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_7", {600.0, 0.0, 0.5, 2000.0, 1000.0});
        this->declare_parameter<std::vector<double>>("position_gains_8", {600.0, 0.0, 0.5, 2000.0, 1000.0});

        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            throw std::runtime_error("Socket creation failed");
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ-1);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ioctl SIOCGIFINDEX failed");
            close(sock_);
            throw std::runtime_error("ioctl failed");
        }
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to interface %s", ifname_.c_str());
            close(sock_);
            throw std::runtime_error("Bind failed");
        }

        target_sub_ = this->create_subscription<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array", 10,
            std::bind(&SenderNode::target_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to motor_cmd_array topic");
        can_sub_ = this->create_subscription<robomas_package_2::msg::CanFrame>("can_frame_tx", 10,
            std::bind(&SenderNode::canframe_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to can_frame_tx topic");
        esc_sub_ = this->create_subscription<robomas_package_2::msg::Esc>("esc_tx", 10,
            std::bind(&SenderNode::esc_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to esc_tx topic");
        RCLCPP_INFO(this->get_logger(), "CAN interface '%s' initialized successfully", ifname_.c_str());
    }

    ~SenderNode() {
        send_canframe(0x001, 8, 0,0,0,0,0,0,0,0);
        for(uint8_t id=1; id<=8; id++){
            send_canframe(0x100 + id, 8, 0,0,0,0,0,0,0,0);
        }
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:

    void target_callback(const robomas_package_2::msg::MotorCmdArray::SharedPtr msg) {
        for(const auto& c : msg->cmds) {
            uint8_t id = c.id;

            if (id < 1 || id > 8) {
                RCLCPP_WARN(this->get_logger(), "Received id out of range: %u", id);
                continue;
            }

            if(c.mode != M[id-1].mode) {
                M[id-1].mode = c.mode;
                std::string name;
                switch(c.mode){
                    case 0:
                        RCLCPP_INFO(this->get_logger(), "Motor %u: Mode changed to CURRENT", id);
                        send_canframe(0x100 + id, 8, 0,0,0,0,0,0,0,0);
                        break;
                    case 1:
                        RCLCPP_INFO(this->get_logger(), "Motor %u: Mode changed to SPEED", id);
                        name = "speed_gains_" + std::to_string(id);
                        set_parameter_and_mode(name, c.mode, id);
                        break;
                    case 2:
                        RCLCPP_INFO(this->get_logger(), "Motor %u: Mode changed to POSITION", id);
                        name = "speed_gains_" + std::to_string(id);
                        set_parameter_and_mode(name, c.mode, id);
                        name = "position_gains_" + std::to_string(id);
                        set_parameter_and_mode(name, c.mode, id);
                        break;
                }
            }

            M[id-1].value = c.value;
            if(c.mode ==2){
                M[id-1].value = c.value * 10;
            }else{
                M[id-1].value = c.value;
            }

            if(1<=id && id<=4){
                uint8_t cmd[8];
                for(int i=0; i<4; i++) {
                    cmd[i*2]   = (uint8_t)((M[i].value >> 8) & 0xFF);
                    cmd[i*2+1] = (uint8_t)(M[i].value & 0xFF);
                }
                send_canframe(0x100, 8, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
            }
            if(5<=id && id<=8){
                uint8_t cmd[8];
                for(int i=0; i<4; i++) {
                    cmd[i*2]   = (uint8_t)((M[i+4].value >> 8) & 0xFF);
                    cmd[i*2+1] = (uint8_t)(M[i+4].value & 0xFF);
                }
                send_canframe(0x0FF, 8, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
            }
        }
    }

    void set_parameter_and_mode(const std::string &name, uint8_t mode, uint8_t id) {
        std::vector<double> gains;
        if (!this->get_parameter(name, gains)) {
            RCLCPP_WARN(this->get_logger(), "Parameter %s not found, using zeros", name.c_str());
            gains = {0,0,0,0,0};
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Parameter %s loaded: [kp=%.1f, ki=%.1f, kd=%.4f, out_max=%.0f, i_sum_max=%.0f]", 
                         name.c_str(), gains[0], gains[1], gains[2], gains[3], gains[4]);
        }
        if (gains.size() < 5) gains.resize(5, 0.0);
        uint8_t cmd[8];
        uint16_t kp = static_cast<uint16_t>(gains[0] * 10);
        uint16_t ki = static_cast<uint16_t>(gains[1] * 10);
        uint16_t kd = static_cast<uint16_t>(gains[2] * 10000);
        uint8_t out_max = static_cast<uint8_t>(gains[3] * 0.01f);
        uint8_t i_sum_max = static_cast<uint8_t>(gains[4] * 0.01f);
        cmd[0] = (kp >> 8) & 0xFF;
        cmd[1] = kp & 0xFF;
        cmd[2] = (ki >> 8) & 0xFF;
        cmd[3] = ki & 0xFF;
        cmd[4] = (kd >> 8) & 0xFF;
        cmd[5] = kd & 0xFF;
        cmd[6] = (out_max) & 0xFF;
        cmd[7] = (mode << 6) | (i_sum_max & 0b00111111);
        
        send_canframe(0x100 + id, 8, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
    }

    void send_canframe(uint16_t id, uint8_t dlc, 
                       uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = id;
        frame.can_dlc = dlc;
        uint8_t data[8] = {d0, d1, d2, d3, d4, d5, d6, d7};
        for(int i = 0; i < dlc; i++){
            frame.data[i] = data[i];
        }
        int nbytes = write(sock_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
        }
    }
    
    void canframe_callback(const robomas_package_2::msg::CanFrame::SharedPtr msg) {
        send_canframe(msg->id, msg->dlc,
                      msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                      msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
    }

    void esc_callback(const robomas_package_2::msg::Esc::SharedPtr msg) {
        if(msg->esc == true){
            send_canframe(0x001, 0, 0,0,0,0,0,0,0,0);
        }else if(msg->esc == false){
            send_canframe(0x002, 0, 0,0,0,0,0,0,0,0);
        }
    }

    std::string ifname_;
    int sock_{-1};
    rclcpp::Subscription<robomas_package_2::msg::MotorCmdArray>::SharedPtr target_sub_;
    rclcpp::Subscription<robomas_package_2::msg::CanFrame>::SharedPtr can_sub_;
    rclcpp::Subscription<robomas_package_2::msg::Esc>::SharedPtr esc_sub_;
    MotorValue M[8];
};

int main(int argc, char * argv[])
{
    /////////////////////////////////////// sender_params.yaml 読み込み begin /////////////////////////////
    // Get the installation directory of the package
    std::string install_prefix;
    const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
    if (ament_prefix_path) {
        install_prefix = std::string(ament_prefix_path);
        // AMENT_PREFIX_PATH might contain multiple paths separated by colons
        size_t first_colon = install_prefix.find(':');
        if (first_colon != std::string::npos) {
            install_prefix = install_prefix.substr(0, first_colon);
        }
    }
    
    // Try to find the params file
    std::string params_file;
    if (!install_prefix.empty()) {
        std::string candidate = install_prefix + "/share/robomas_package_2/sender_params.yaml";
        if (std::filesystem::exists(candidate)) {
            params_file = candidate;
        }
    }
    
    // Also check in current directory as fallback
    if (params_file.empty()) {
        std::string cwd_candidate = "./src/robomas_package_2/sender_params.yaml";
        if (std::filesystem::exists(cwd_candidate)) {
            params_file = cwd_candidate;
        }
    }
    
    // Build argv with params file if found
    std::vector<char*> new_argv;
    for (int i = 0; i < argc; ++i) {
        new_argv.push_back(argv[i]);
    }
    
    std::string params_arg = "--ros-args";
    if (!params_file.empty()) {
        new_argv.push_back(const_cast<char*>(params_arg.c_str()));
        new_argv.push_back(const_cast<char*>("--params-file"));
        new_argv.push_back(const_cast<char*>(params_file.c_str()));
    }

    /////////////////////////////////////// sender_params.yaml 読み込み end /////////////////////////////
    
    rclcpp::init(new_argv.size(), new_argv.data());
    
    // Create a temporary logger just for startup info
    auto temp_node = std::make_shared<rclcpp::Node>("_temp");
    if (!params_file.empty()) {
        RCLCPP_INFO(temp_node->get_logger(), "Parameter file loaded: %s", params_file.c_str());
    } else {
        RCLCPP_WARN(temp_node->get_logger(), "No parameter file found, using default parameters");
    }
    temp_node.reset();
    
    auto node = std::make_shared<SenderNode>("can0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}