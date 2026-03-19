#include "agv_hardware/agv_hardware.hpp"
#include <jsoncpp/json/reader.h>

namespace agv_hardware {

hardware_interface::CallbackReturn AgvHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    remoteIP = "192.168.1.159";
    localIP = "192.168.1.100";

    robot_ = std::make_shared<rokae::xMateRobot>();

    // 初始化 WebSocket 客户端
    initWebSocket();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    try {
        robot_->connectToRobot(remoteIP, localIP);

        // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
        //             "Connected to robot at %s", remoteIP.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch (const rokae::NetworkException& e) {
        // RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
        //              "Network error connecting to robot at %s: %s",
        //              remoteIP.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    catch (const rokae::ExecutionException& e) {
        // RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
        //              "Execution error connecting to robot at %s: %s",
        //              remoteIP.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 注意：is_rt_mode_ 需要在调用 setMotionControlMode 之前由外部配置决定
    // 这里默认使用实时模式，如果需要非实时模式，请修改 is_rt_mode_ = false;
    
    // 设置网络容差
    robot_->setRtNetworkTolerance(20, ec);

    if (is_rt_mode_) {
        // === 实时模式初始化 ===
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "正在初始化实时模式...");
        
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        robot_->setPowerState(true, ec);

        // 启动状态接收
        robot_->startReceiveRobotState(std::chrono::milliseconds(1),
            {rokae::RtSupportedFields::jointPos_m});

        rt_controller_ = robot_->getRtMotionController().lock();
        if (!rt_controller_) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                         "获取 RT 控制器失败");
            return hardware_interface::CallbackReturn::ERROR;
        }

        rt_controller_->startMove(rokae::RtControllerMode::jointPosition);

        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "实时模式激活完成，RT 循环已启动");
    } else {
        // === 非实时模式初始化 ===
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "正在初始化非实时模式...");
        
        robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        robot_->setPowerState(true, ec);
        
        // 非实时模式需要设置默认速度
        robot_->setDefaultSpeed(300.0, ec);  // 200mm/s
        robot_->adjustSpeedOnline(0.2,ec);
        robot_->moveReset(ec);  // 重置运动队列
        
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "非实时模式激活完成");
    }

    // 读取初始关节位置
    joint_positions_ = robot_->jointPos(ec);

    // 连接 WebSocket 服务器
    connectWebSocket();

    // 启动本地 WebSocket 服务器
    startLocalWebSocketServer();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    // 断开 WebSocket 连接
    disconnectWebSocket();

    if (is_rt_mode_) {
        // 实时模式清理
        if (rt_controller_) {
            rt_controller_->stopLoop();
            rt_controller_->stopMove();
            rt_controller_.reset();
        }
    } else {
        // 非实时模式清理
        robot_->stop(ec);
        robot_->moveReset(ec);
    }

    robot_->stopReceiveRobotState();
    robot_->setPowerState(false, ec);
    robot_->disconnectFromRobot(ec);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AgvHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    
    // 读取当前关节位置（两种模式都适用）
    joint_positions_ = robot_->jointPos(ec);
    
    // 更新状态接口
    set_state("arm1_joint1/position", joint_positions_[0]);
    set_state("arm1_joint2/position", joint_positions_[1]);
    set_state("arm1_joint3/position", joint_positions_[2]);
    set_state("arm1_joint4/position", joint_positions_[3]);
    set_state("arm1_joint5/position", joint_positions_[4]);
    set_state("arm1_joint6/position", joint_positions_[5]);

    // 发送关节位置到 WebSocket 服务器
    sendJointData(joint_positions_);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    std::lock_guard<std::mutex> lock(io_mutex);

    // 读取控制器命令
    double cmds[6] = {
        get_command("arm1_joint1/position"),
        get_command("arm1_joint2/position"),
        get_command("arm1_joint3/position"),
        get_command("arm1_joint4/position"),
        get_command("arm1_joint5/position"),
        get_command("arm1_joint6/position")
    };

    // 清洗：若非有限值则回退到当前状态
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(cmds[i])) {
            cmds[i] = joint_positions_[i];  // 使用当前角度作为安全值
        }
        joint_commands_[i] = cmds[i];
    }

    if (is_rt_mode_) {
        // === 实时模式：RT 控制循环 ===
        if (callback_init == 0)
        {
            rt_controller_->setControlLoop<rokae::JointPosition>(
                [this]() -> rokae::JointPosition {
                    rokae::JointPosition cmd;
                    cmd.joints.resize(6);

                    // 每次回调都读取当前角度
                    std::array<double, 6> current = robot_->jointPos(ec);

                    for (size_t i = 0; i < 6; ++i) {
                        joint_positions_[i] = current[i];
                        // 防御：若命令非有限值则回退到当前角度
                        cmd.joints[i] = std::isfinite(joint_commands_[i]) ?
                                        joint_commands_[i] : current[i];
                    }
                    return cmd;
                },
                0,
                true
            );

            rt_controller_->startLoop(false); // 启动循环（非阻塞）
            callback_init = 1;
            
            // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
            //             "RT 控制循环已启动");
        }
    }
    // 非实时模式：write() 仅更新命令缓存，不执行运动命令
    // 非实时命令在 parseAndExecuteCommand() 中直接执行

    return hardware_interface::return_type::OK;
}

// ============ 非实时模式辅助函数 ============

void AgvHardwareInterface::sendNrtJointCommand(const std::array<double, 6>& joints)
{
//     error_code ec;
//     std::string id;
//     rokae::MoveJCommand moveJ({0.2434, -0.314, 0.591, 1.5456, 0.314, 2.173});
//   // 同样的末端位姿，confData不同，轴角度也不同
//   // the target posture is same, but give different joint configure data
//   // moveJ.target.confData =  {-67, 100, 88, -79, 90, -120, 0, 0};
//   // 示例：设置关节速度百分比为10%。如不设置的话，关节速度根据末端线速度计算得出
//     robot_->moveAppend({moveJ}, id, ec);

//     // moveJ.target.confData =  {-76, 8, -133, -106, 103, 108, 0, 0};
//     // robot.moveAppend({moveJ}, id, ec);
//     // moveJ.target.confData =  {-70, 8, -88, 90, -105, -25, 0, 0};
//     // robot.moveAppend({moveJ}, id, ec);

//     robot_->moveStart(ec);
//     waitForFinish(robot_, id, 0);
    try {
        // 参数检查
        for (size_t i = 0; i < 6; ++i) {
            if (joints[i] < -3.14 || joints[i] > 3.14) {
                RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                           "关节 %zu 角度 %.3f 可能超出范围", i, joints[i]);
            }
        }
        
        rokae::JointPosition target;
        target.joints = std::vector<double>(joints.begin(), joints.end());
        
        // 使用合理的参数
        rokae::MoveAbsJCommand cmd(target);
        // cmd.speed = 500.0;   // 30% 速度，更安全
        // cmd.zone = 50;       // 5mm 转弯区
        
        std::string cmd_id;
        ec.clear();
        
        robot_->moveAppend(cmd, cmd_id, ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "moveAppend失败: %s", ec.message().c_str());
            return;
        }
        
        nrt_executing_ = true;
        current_cmd_id_ = cmd_id;
        
        ec.clear();
        robot_->moveStart(ec);
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "moveStart失败: %s", ec.message().c_str());
            nrt_executing_ = false;
            return;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                   "发送关节命令成功: ID=%s", cmd_id.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "异常: %s", e.what());
        nrt_executing_ = false;
    }
}

void AgvHardwareInterface::sendNrtCartesianCommand(
    const std::vector<std::array<double, 6>>& points,
    const std::string& move_type, double speed)
{
    if (nrt_executing_) {
        RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                   "当前有运动正在执行，忽略新命令");
        return;
    }
    
    try {
        robot_->moveReset(ec);  // 重置队列
        
        for (const auto& point : points) {
            if (move_type == "joint") {
                // 关节空间运动
                rokae::JointPosition target;
                target.joints = std::vector<double>(point.begin(), point.end());
                rokae::MoveAbsJCommand cmd(target);
                // 打印目标关节位置
                std::stringstream joint_ss;
                joint_ss << "目标关节位置: [";
                for (size_t j = 0; j < target.joints.size(); ++j) {
                    if (j > 0) joint_ss << ", ";
                    joint_ss << target.joints[j];
                }
                joint_ss << "]";
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"), "%s", joint_ss.str().c_str());
                
                // 打印命令参数
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"), 
                            "命令类型: MoveAbsJCommand");
                cmd.speed = speed > 0 ? speed : 300.0;
                cmd.zone = 50;
                // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                robot_->moveAppend(cmd, current_cmd_id_, ec);
            } else {
                // 笛卡尔空间运动 (point 存储的是 x,y,z,rx,ry,rz)
                // Frame 使用 trans 和 rpy 成员
                rokae::CartesianPosition target;
                target.trans[0] = point[0];  // X
                target.trans[1] = point[1];  // Y
                target.trans[2] = point[2];  // Z
                target.rpy[0] = point[3];    // Rx
                target.rpy[1] = point[4];    // Ry
                target.rpy[2] = point[5];    // Rz
                
                rokae::MoveLCommand cmd(target);
                
                cmd.speed = speed > 0 ? speed : 100.0;  // mm/s
                cmd.zone = 0.5;  // mm
                robot_->moveAppend(cmd, current_cmd_id_, ec);
            }
            
            if (ec) {
                RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                            "添加笛卡尔命令失败：%s", ec.message().c_str());
                return;
            }
        }
        
        robot_->moveStart(ec);
        nrt_executing_ = true;
        
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                   "发送非实时笛卡尔命令：点数=%zu, 类型=%s, 速度=%.1f",
                   points.size(), move_type.c_str(), speed);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "发送非实时笛卡尔命令异常：%s", e.what());
    }
}

void AgvHardwareInterface::waitForMotionComplete(int timeout_ms)
{
    auto start = std::chrono::steady_clock::now();
    
    while (nrt_executing_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // 检查机器人实际状态
        try {
            ec.clear();
            auto state = robot_->operationState(ec);
            if (!ec) {
                if (state == rokae::OperationState::idle) {
                    RCLCPP_DEBUG(rclcpp::get_logger("AgvHardwareInterface"),
                                "运动完成");
                    nrt_executing_ = false;
                    break;
                }
            }
        } catch (...) {
            // 忽略异常
        }
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "等待超时 %dms", timeout_ms);
            // 尝试停止运动
            robot_->stop(ec);
            nrt_executing_ = false;
            break;
        }
    }
}

// ============ WebSocket 客户端实现 ============

void AgvHardwareInterface::initWebSocket()
{
    try {
        // 创建 WebSocket 客户端
        ws_client_ = std::make_unique<WebSocketClient>();

        // 初始化 ASIO
        ws_client_->init_asio();

        // 设置日志级别（调试时可打开）
        ws_client_->clear_access_channels(websocketpp::log::alevel::all);
        ws_client_->set_access_channels(websocketpp::log::alevel::connect |
                                       websocketpp::log::alevel::disconnect |
                                       websocketpp::log::alevel::app);

        ws_client_->clear_error_channels(websocketpp::log::elevel::all);
        ws_client_->set_error_channels(websocketpp::log::elevel::rerror |
                                      websocketpp::log::elevel::fatal);

        // 设置处理器
        ws_client_->set_open_handler(std::bind(&AgvHardwareInterface::onOpen, this, std::placeholders::_1));
        ws_client_->set_fail_handler(std::bind(&AgvHardwareInterface::onFail, this, std::placeholders::_1));
        ws_client_->set_close_handler(std::bind(&AgvHardwareInterface::onClose, this, std::placeholders::_1));
        ws_client_->set_message_handler(std::bind(&AgvHardwareInterface::onMessage, this,
            std::placeholders::_1, std::placeholders::_2));

        ws_running_ = true;

        // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
        //            "WebSocket 客户端初始化完成");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "WebSocket 初始化失败：%s", e.what());
    }
}

void AgvHardwareInterface::connectWebSocket()
{
    // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
    //            "connectWebSocket() 被调用，连接到：%s", WS_SERVER_URI.c_str());

    if (!ws_client_) {
        // RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
        //             "WebSocket 客户端未初始化");
        return;
    }

    // 如果已经有连接线程在运行，先停止
    if (ws_thread_ && ws_thread_->joinable()) {
        disconnectWebSocket();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ws_running_ = true;
    ws_connected_ = false;

    // 在独立线程中运行 WebSocket 客户端
    ws_thread_ = std::make_unique<std::thread>([this]() {
        // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
        //            "WebSocket 连接线程启动");

        while (ws_running_) {
            try {
                websocketpp::lib::error_code ec;

                // 创建连接对象
                WebSocketConnectionPtr con = ws_client_->get_connection(WS_SERVER_URI, ec);
                if (!ec) {
                    // 设置连接超时（默认可能太短）
                    con->set_open_handshake_timeout(5000);  // 5 秒
                    con->set_close_handshake_timeout(3000); // 3 秒
                }
                if (ec) {
                    // RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    //            "创建连接失败：%s", ec.message().c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(WS_RECONNECT_INTERVAL));
                    continue;
                }

                // 设置连接参数
                con->append_header("User-Agent", "AGV-Hardware-Client/1.0");

                // 保存连接句柄
                connection_hdl_ = con->get_handle();

                // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                //            "正在连接到服务器...");

                // 连接到服务器
                ws_client_->connect(con);

                // 运行事件循环（这里会阻塞直到连接断开）
                ws_client_->run();

                // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                //            "连接断开，准备重连...");

            }
            catch (const websocketpp::exception& e) {
                RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                          "WebSocket 异常：%s", e.what());
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                           "连接线程异常：%s", e.what());
            }

            // 如果还在运行状态，等待后重连
            if (ws_running_) {
                // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                //            "等待 %dms 后重连...", WS_RECONNECT_INTERVAL);
                std::this_thread::sleep_for(std::chrono::milliseconds(WS_RECONNECT_INTERVAL));

                // 重置客户端以重新连接
                try {
                    if (ws_client_) {
                        ws_client_->reset();
                        ws_client_->init_asio();

                        // 重新设置处理器
                        ws_client_->set_open_handler(std::bind(&AgvHardwareInterface::onOpen, this, std::placeholders::_1));
                        ws_client_->set_fail_handler(std::bind(&AgvHardwareInterface::onFail, this, std::placeholders::_1));
                        ws_client_->set_close_handler(std::bind(&AgvHardwareInterface::onClose, this, std::placeholders::_1));
                        ws_client_->set_message_handler(std::bind(&AgvHardwareInterface::onMessage, this,
                            std::placeholders::_1, std::placeholders::_2));
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                              "重置客户端失败：%s", e.what());
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                   "WebSocket 连接线程退出");
    });
}

void AgvHardwareInterface::onOpen(websocketpp::connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(ws_mutex_);
    ws_connected_ = true;
    connection_hdl_ = hdl;

    // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
    //            "WebSocket 连接成功");

    // 发送连接确认消息
    try {
        Json::Value root;
        root["type"] = "connection";
        root["client"] = "agv_hardware";
        root["timestamp"] = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

        Json::StreamWriterBuilder writer;
        std::string json_str = Json::writeString(writer, root);

        websocketpp::lib::error_code ec;
        ws_client_->send(hdl, json_str, websocketpp::frame::opcode::text, ec);

        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "发送连接确认失败：%s", ec.message().c_str());
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "发送连接确认异常：%s", e.what());
    }
}

void AgvHardwareInterface::onFail(websocketpp::connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(ws_mutex_);
    ws_connected_ = false;

    WebSocketClient::connection_ptr con = ws_client_->get_con_from_hdl(hdl);
    // RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
    //            "WebSocket 连接失败：%s", con->get_ec().message().c_str());
}

void AgvHardwareInterface::onClose(websocketpp::connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(ws_mutex_);
    ws_connected_ = false;

    WebSocketClient::connection_ptr con = ws_client_->get_con_from_hdl(hdl);
    // RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
    //            "WebSocket 连接关闭：%s", con->get_remote_close_reason().c_str());
}

void AgvHardwareInterface::startLocalWebSocketServer()
{
    // 创建一个简单的服务器线程
    std::thread([this]() {
        try {
            WebSocketServer server;

            // 初始化服务器
            server.init_asio();

            // 设置日志级别
            server.clear_access_channels(websocketpp::log::alevel::all);
            server.set_access_channels(websocketpp::log::alevel::connect |
                                      websocketpp::log::alevel::disconnect);

            server.clear_error_channels(websocketpp::log::elevel::all);

            // 设置消息处理器
            server.set_message_handler([this](websocketpp::connection_hdl hdl,
                                             WebSocketServerMsgPtr msg) {
                (void)hdl;  // 避免未使用参数警告
                try {
                    std::string payload = msg->get_payload();
                    RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                               "收到本地控制指令：%s", payload.c_str());

                    // 直接调用已有的命令解析函数
                    parseAndExecuteCommand(payload);
                }
                catch (const std::exception& e) {
                    RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                               "处理本地消息异常：%s", e.what());
                }
            });

            // 设置连接处理器
            server.set_open_handler([this, &server](websocketpp::connection_hdl hdl) {
                WebSocketServer::connection_ptr con = server.get_con_from_hdl(hdl);
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                           "本地客户端连接：%s", con->get_remote_endpoint().c_str());
            });

            server.set_close_handler([](websocketpp::connection_hdl hdl) {
                (void)hdl;  // 避免未使用参数警告
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                           "本地客户端断开连接");
            });

            // 监听端口
            server.listen(WS_LOCAL_SERVER_PORT);
            server.start_accept();

            RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                       "本地 WebSocket 服务器已启动，监听端口：%d", WS_LOCAL_SERVER_PORT);

            // 运行服务器
            server.run();

        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "本地 WebSocket 服务器异常：%s", e.what());
        }
    }).detach();  // 分离线程，让它在后台运行
}

void AgvHardwareInterface::onMessage(websocketpp::connection_hdl hdl, WebSocketMessagePtr msg)
{
    (void)hdl;

    try {
        std::string payload = msg->get_payload();
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                    "收到服务器指令：%s", payload.c_str());

        // 解析并执行命令
        parseAndExecuteCommand(payload);
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                   "处理消息异常：%s", e.what());
    }
}

void AgvHardwareInterface::disconnectWebSocket()
{
    RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
               "断开 WebSocket 连接...");

    ws_running_ = false;

    // 关闭连接
    if (ws_client_ && ws_connected_) {
        try {
            websocketpp::lib::error_code ec;
            ws_client_->close(connection_hdl_,
                             websocketpp::close::status::normal,
                             "Client shutdown", ec);

            if (ec) {
                RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                           "关闭连接失败：%s", ec.message().c_str());
            }
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "关闭连接异常：%s", e.what());
        }
    }

    // 停止客户端
    if (ws_client_) {
        try {
            ws_client_->stop();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "停止客户端异常：%s", e.what());
        }
    }

    // 等待线程结束
    if (ws_thread_ && ws_thread_->joinable()) {
        try {
            ws_thread_->join();
            ws_thread_.reset();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "等待线程结束时出错：%s", e.what());
        }
    }

    ws_connected_ = false;

    RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
               "WebSocket 客户端已断开");
}

void AgvHardwareInterface::sendJointData(const std::array<double, 6>& positions)
{
    if (!ws_connected_) {
        return;
    }

    try {
        // 创建 JSON 数据
        Json::Value root;
        root["target"] = "arm_left";

        Json::Value data;
        data["joint1"] = positions[0];
        data["joint2"] = positions[1];
        data["joint3"] = positions[2];
        data["joint4"] = positions[3];
        data["joint5"] = positions[4];
        data["joint6"] = positions[5];

        root["data"] = data;

        // 添加时间戳
        root["timestamp"] = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

        // 转换为字符串
        Json::StreamWriterBuilder writer;
        std::string json_str = Json::writeString(writer, root);

        // 发送数据
        websocketpp::lib::error_code ec;
        ws_client_->send(connection_hdl_, json_str, websocketpp::frame::opcode::text, ec);

        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "发送失败：%s", ec.message().c_str());
            ws_connected_ = false;
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "发送关节数据异常：%s", e.what());
        ws_connected_ = false;
    }
}

// 解析并执行从服务器接收到的命令
void AgvHardwareInterface::parseAndExecuteCommand(const std::string& json_str)
{
    try {
        Json::Value root;
        Json::CharReaderBuilder reader;
        std::string errors;
        std::istringstream json_stream(json_str);

        if (!Json::parseFromStream(reader, json_stream, &root, &errors)) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "JSON 解析错误：%s", errors.c_str());
            return;
        }

        // 如果输入是数组，取第一个元素
        if (root.isArray()) {
            if (root.size() > 0) {
                root = root[0];
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                            "JSON 数组为空");
                return;
            }
        }

        // 获取目标设备
        std::string target = root.get("target", "").asString();
        if (target.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "缺少 target 字段");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                   "接收到命令，目标设备：%s", target.c_str());

        // 获取数据部分
        Json::Value data = root["data"];
        if (!data.isObject()) {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "数据格式错误：data 字段不是对象");
            return;
        }

        std::vector<std::array<double, 6>> path_points;

        // 情况 1：笛卡尔路径点
        if (data.isMember("move_point")) {
            Json::Value move_points = data["move_point"];
            if (!move_points.isArray()) {
                RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                            "数据格式错误：move_point 不是数组");
                return;
            }

            for (Json::ArrayIndex i = 0; i < move_points.size(); ++i) {
                Json::Value point = move_points[i];
                if (!point.isObject()) {
                    RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                               "路径点 %u 格式错误，跳过", i);
                    continue;
                }

                std::array<double, 6> point_data;
                point_data[0] = point.get("x", 0.0).asDouble();
                point_data[1] = point.get("y", 0.0).asDouble();
                point_data[2] = point.get("z", 0.0).asDouble();
                point_data[3] = point.get("rx", 0.0).asDouble();
                point_data[4] = point.get("ry", 0.0).asDouble();
                point_data[5] = point.get("rz", 0.0).asDouble();

                path_points.push_back(point_data);
            }
        }
        // 情况 2：关节指令 val1~val6
        else if (data.isMember("val1")) {
            std::array<double, 6> joint_data;
            joint_data[0] = data.get("val1", 0.0).asDouble();
            joint_data[1] = data.get("val2", 0.0).asDouble();
            joint_data[2] = data.get("val3", 0.0).asDouble();
            joint_data[3] = data.get("val4", 0.0).asDouble();
            joint_data[4] = data.get("val5", 0.0).asDouble();
            joint_data[5] = data.get("val6", 0.0).asDouble();

            path_points.push_back(joint_data);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                        "数据格式错误：缺少 move_point 或 val1~val6 字段");
            return;
        }

        if (path_points.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("AgvHardwareInterface"),
                       "没有有效的路径点");
            return;
        }

        // 获取其他参数
        double speed = data.get("speed", 0.0).asDouble();
        bool wait_for_completion = data.get("wait", true).asBool();

        // 优先使用 control_type 覆盖 move_type
        std::string move_type = data.get("move_type", "").asString();
        std::string control_type = data.get("control_type", "").asString();
        if (!control_type.empty()) {
            move_type = control_type;
        }
        if (move_type.empty()) {
            move_type = "linear";  // 默认值
        }

        {
            std::lock_guard<std::mutex> lock(io_mutex);
            cartesian_path_points_ = path_points;
            current_path_index_ = 0;
            cartesian_speed_ = speed;
            move_type_ = move_type;
            execute_cartesian_path_ = true;

            if (move_type == "joint") {
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                           "接收到关节空间路径规划，点数：%zu, 速度：%.1f",
                           path_points.size(), speed);
            } else {
                RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                           "接收到笛卡尔空间路径规划，点数：%zu, 速度：%.1f, 类型：%s",
                           path_points.size(), speed, move_type.c_str());
            }
        }

        // 根据模式执行路径
        if (is_rt_mode_) {
            // 实时模式：设置标志位，由 write() 中的 RT 循环处理
            RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                       "实时模式：路径将由 RT 循环执行");
        } else {
            // 非实时模式：直接发送运动指令
            RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                       "非实时模式：直接执行路径");
            
            if (move_type == "joint") {
                // 关节空间路径：逐点发送
                for (size_t i = 0; i < path_points.size(); ++i) {
                    RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                               "执行关节点 %zu/%zu", i+1, path_points.size());
                    sendNrtJointCommand(path_points[i]);
                    
                    // 如果需要等待完成
                    if (wait_for_completion) {
                        waitForMotionComplete();
                    }
                }
            } else {
                // 笛卡尔空间路径
                sendNrtCartesianCommand(path_points, move_type, speed);
                
                // 如果需要等待完成
                if (wait_for_completion) {
                    waitForMotionComplete();
                }
            }
        }

        if (wait_for_completion) {
            RCLCPP_INFO(rclcpp::get_logger("AgvHardwareInterface"),
                       "等待路径执行完成");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AgvHardwareInterface"),
                    "解析命令时发生异常：%s", e.what());
    }
}

} // namespace agv_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    agv_hardware::AgvHardwareInterface,
    hardware_interface::SystemInterface)
