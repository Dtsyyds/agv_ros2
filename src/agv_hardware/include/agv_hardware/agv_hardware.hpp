#ifndef AGV_HARDWARE_HPP
#define AGV_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include "rokae/robot.h"
#include "rokae/data_types.h"


#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif

#ifndef BOOST_ERROR_CODE_HEADER_ONLY
#define BOOST_ERROR_CODE_HEADER_ONLY
#endif

// 使用非 TLS 版本的 WebSocket
#include <websocketpp/client.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/config/asio.hpp>

// ASIO
#ifdef ASIO_STANDALONE
#include <asio.hpp>
#else
#include <boost/asio.hpp>
#endif

#include <jsoncpp/json/json.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <functional>

namespace agv_hardware {

// 使用非 TLS 配置
using WebSocketConfig = websocketpp::config::asio_client;
using WebSocketClient = websocketpp::client<WebSocketConfig>;
using WebSocketServer = websocketpp::server<websocketpp::config::asio>;
using WebSocketMessagePtr = WebSocketClient::message_ptr;
using WebSocketConnectionPtr = WebSocketClient::connection_ptr;
using WebSocketServerMsgPtr = WebSocketServer::message_ptr;

class AgvHardwareInterface : public hardware_interface::SystemInterface {
public:
    // lifecycle node overrides
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface overrides
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & params) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // WebSocket 客户端相关函数
    void initWebSocket();
    void connectWebSocket();
    void disconnectWebSocket();
    void sendJointData(const std::array<double, 6>& positions);
    void onOpen(websocketpp::connection_hdl hdl);
    void onFail(websocketpp::connection_hdl hdl);
    void onClose(websocketpp::connection_hdl hdl);
    void onMessage(websocketpp::connection_hdl hdl, WebSocketMessagePtr msg);
    void parseAndExecuteCommand(const std::string& json_str);

    // 服务器初始化函数
    void startLocalWebSocketServer();

    // 非实时模式辅助函数
    void sendNrtJointCommand(const std::array<double, 6>& joints);
    void sendNrtCartesianCommand(const std::vector<std::array<double, 6>>& points, 
                                  const std::string& move_type, double speed);
    void waitForMotionComplete(int timeout_ms = 10000);
    

    // WebSocket 客户端成员
    std::unique_ptr<WebSocketClient> ws_client_;
    std::unique_ptr<std::thread> ws_thread_;

    std::atomic<bool> ws_connected_{false};
    std::atomic<bool> ws_running_{false};

    websocketpp::connection_hdl connection_hdl_;
    std::mutex ws_mutex_;

    // WebSocket 配置
    const std::string WS_SERVER_URI = "ws://192.168.1.163:9000";
    const int WS_LOCAL_SERVER_PORT = 9001;
    const int WS_RECONNECT_INTERVAL = 5000;

    std::string remoteIP;
    std::string localIP;
    error_code ec;
    std::shared_ptr<rokae::xMateRobot> robot_;

    std::array<double, 6> joint_positions_{};
    std::array<double, 6> joint_commands_{};
    std::mutex io_mutex;

    std::shared_ptr<rokae::RtMotionControlCobot<6>> rt_controller_;

    bool callback_init = 0;
    
    // 新增：模式标记
    bool is_rt_mode_ = false;  // true=实时模式，false=非实时模式
    
    // 非实时模式执行状态
    std::atomic<bool> nrt_executing_ = false;
    std::string current_cmd_id_;

    // 笛卡尔路径规划相关
    std::vector<std::array<double, 6>> cartesian_path_points_;
    size_t current_path_index_ = 0;
    bool execute_cartesian_path_ = false;
    double cartesian_speed_ = 0.0;
    std::string move_type_ = "linear";
};

}; // namespace agv_hardware

#endif // AGV_HARDWARE_HPP
