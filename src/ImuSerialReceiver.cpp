#include "imu_serial_reader/ImuSerialReceiver.hpp"

ImuSerialReceiver::ImuSerialReceiver(const std::string & port, unsigned int baud_rate)
    : Node("imu_serial_receiver_node"),
    port_name_(port),
    baud_rate_(baud_rate),
    io_(),
    serial_(io_),
    running(false)
{
    initParameters();
}

void ImuSerialReceiver::initParameters()
{
    // 파라미터 선언
    this->declare_parameter<std::string>("device_port", "/dev/bno_imu");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<std::string>("topic_name_imu_data", "/imu/data");
    this->declare_parameter<std::string>("topic_name_gravity", "/imu/gravity");
    // 파라미터 가져오기 (필터 알파값은 기본값으로 초기화 후 setupSliderParameter에서 선언)
    filter_alpha_acc_ = 0.1;
    filter_alpha_gyro_ = 0.1;
    filter_alpha_gravity_ = 0.1;
    filter_alpha_quat_ = 0.1;

    // 슬라이더 파라미터 설정 (rqt_reconfigure에서 슬라이더로 표시됨)
    setupSliderParameter();

    // 파라미터 가져오기
    device_port_ = this->get_parameter("device_port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    topic_name_imu_data_ = this->get_parameter("topic_name_imu_data").as_string();
    topic_name_gravity_ = this->get_parameter("topic_name_gravity").as_string();
    filter_alpha_acc_ = this->get_parameter("filter_alpha_acc").as_double();
    filter_alpha_gyro_ = this->get_parameter("filter_alpha_gyro").as_double();
    filter_alpha_gravity_ = this->get_parameter("filter_alpha_gravity").as_double();
    filter_alpha_quat_ = this->get_parameter("filter_alpha_quat").as_double();

    RCLCPP_INFO(this->get_logger(), "IMU Serial Receiver Node initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "Device Port: %s", device_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud Rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Topic Name IMU Data: %s", topic_name_imu_data_.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic Name Gravity: %s", topic_name_gravity_.c_str());
    RCLCPP_INFO(this->get_logger(), "Filter Alpha Acc: %f", filter_alpha_acc_);
    RCLCPP_INFO(this->get_logger(), "Filter Alpha Gyro: %f", filter_alpha_gyro_);
    RCLCPP_INFO(this->get_logger(), "Filter Alpha Gravity: %f", filter_alpha_gravity_);
    RCLCPP_INFO(this->get_logger(), "Filter Alpha Quat: %f", filter_alpha_quat_);
}

ImuSerialReceiver::~ImuSerialReceiver()
{
    if (serial_.is_open())
    {
        serial_.close();
    }
}

void ImuSerialReceiver::openPort()
{
    boost::system::error_code ec;

    serial_.open(port_name_, ec);
    if (ec) { throw std::runtime_error("Failed to open serial port: " + ec.message()); }

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    boost::asio::serial_port_base::baud_rate current_baud;
    serial_.get_option(current_baud);
    RCLCPP_INFO(this->get_logger(), "Serial port configured: Requested=%u, Current=%u", baud_rate_, current_baud.value());

    running = true;
    read_thread = std::thread(&ImuSerialReceiver::readLoop, this);

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s at %u baud", port_name_.c_str(), baud_rate_);
}

void ImuSerialReceiver::closePort()
{
    running = false;
    if (read_thread.joinable())
    {
        read_thread.join();
    }
    if (serial_.is_open())
    {
        serial_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Serial port closed: %s", port_name_.c_str());
}

void ImuSerialReceiver::readLoop()
{
    std::vector<uint8_t> temp_buffer(256);
    RCLCPP_INFO(this->get_logger(), "Starting IMU data read loop...");
    data_buffer_.clear();
    data_buffer_.reserve(512);

    size_t packet_count = 0;
    size_t error_count = 0;

    while (running)
    {
        boost::system::error_code ec;
        size_t len = serial_.read_some(boost::asio::buffer(temp_buffer), ec);

        if(!ec && len > 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes from serial port (buffer size: %zu)", len, data_buffer_.size());

            data_buffer_.insert(data_buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + len);

            const size_t PACKET_SIZE = 67;  // 2 + 4*4 + 3*4 + 3*4 + 3*4 + 3*4 + 1 = 67
            while(data_buffer_.size() >= PACKET_SIZE)
            {
                bool found = false;
                size_t search_limit = data_buffer_.size() - PACKET_SIZE + 1;

                for(size_t i = 0; i < search_limit; i++)
                {
                    if(data_buffer_[i] == 0xAA && data_buffer_[i+1] == 0x55)
                    {
                        std::vector<uint8_t> packet(data_buffer_.begin() + i, data_buffer_.begin() + i + PACKET_SIZE);

                        IMU_Packet_t parsed;
                        if(parseFrame(packet, parsed))
                        {
                            {
                                std::unique_lock<std::shared_mutex> lk(sensor_mtx_);
                                sensor_data = parsed;
                            }
                            data_ready_.store(true);

                            packet_count++;

                            if(packet_count % 20 == 0)
                            {
                                RCLCPP_INFO(this->get_logger(),
                                    "[Packet #%zu] Quat: [%.3f, %.3f, %.3f, %.3f] | "
                                    "RPY: [%.3f, %.3f, %.3f] deg | "
                                    "Acc: [%.2f, %.2f, %.2f] m/s² | "
                                    "Gravity: [%.2f, %.2f, %.2f] m/s²",
                                    packet_count,
                                    sensor_data.quat[0], sensor_data.quat[1], sensor_data.quat[2], sensor_data.quat[3],
                                    sensor_data.rpy[0], sensor_data.rpy[1], sensor_data.rpy[2],
                                    sensor_data.acc[0], sensor_data.acc[1], sensor_data.acc[2],
                                    sensor_data.gravity[0], sensor_data.gravity[1], sensor_data.gravity[2]);
                            }

                            data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + i + PACKET_SIZE);
                            found = true;
                            break;
                        }
                        else
                        {
                            RCLCPP_DEBUG(this->get_logger(), "Invalid packet at offset %zu, skipping header", i);
                            data_buffer_.erase(data_buffer_.begin() + i, data_buffer_.begin() + i + 1);
                            found = true;
                            break;
                        }
                    }
                }

                if(!found)
                {
                    if(data_buffer_.size() >= PACKET_SIZE)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "No header found in buffer (%zu bytes), removing 1 byte", data_buffer_.size());
                        data_buffer_.erase(data_buffer_.begin());
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if(data_buffer_.size() > 512)
            {
                error_count++;
                RCLCPP_WARN(this->get_logger(), "Buffer overflow (#%zu), clearing old data. Size: %zu bytes",
                           error_count, data_buffer_.size());
                data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + (data_buffer_.size() - 256));
            }
        }
        else if(ec)
        {
            error_count++;
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
            running = false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Read loop terminated. Total packets: %zu, Errors: %zu", packet_count, error_count);
}

bool ImuSerialReceiver::parseFrame(const std::vector<uint8_t>& packet, IMU_Packet_t& data_out)
{
    const size_t expectedLength = sizeof(IMU_Packet_t);
    if (packet.size() != expectedLength)
    {
        RCLCPP_WARN(this->get_logger(), "Packet size error: Expected %zu, got %zu", expectedLength, packet.size());
        return false;
    }

    if (packet[0] != 0xAA || packet[1] != 0x55)
    {
        RCLCPP_WARN(this->get_logger(), "Packet header error: Got 0x%02X 0x%02X (expected 0xAA 0x55)", packet[0], packet[1]);
        return false;
    }

    uint8_t calculated_sum = 0;
    for(size_t i = 0; i < expectedLength - 1; i++)
    {
        calculated_sum += packet[i];
    }

    uint8_t received_checksum = packet[expectedLength - 1];

    if(calculated_sum != received_checksum)
    {
        RCLCPP_WARN(this->get_logger(), "Packet checksum error: Calc(0x%02X) != Recv(0x%02X)", calculated_sum, received_checksum);
        return false;
    }

    memcpy(&data_out, packet.data(), expectedLength);
    RCLCPP_DEBUG(this->get_logger(), "[Packet] Successfully parsed and validated (checksum: 0x%02X)", received_checksum);

    return true;
}

void ImuSerialReceiver::setupSliderParameter()
{
    // filter_alpha_acc 슬라이더
    rcl_interfaces::msg::ParameterDescriptor desc_acc;
    desc_acc.description = "가속도 로우패스 필터 알파값 (0.0~1.0)";
    rcl_interfaces::msg::FloatingPointRange range_acc;
    range_acc.from_value = 0.0;
    range_acc.to_value = 1.0;
    range_acc.step = 0.01;
    desc_acc.floating_point_range.push_back(range_acc);
    this->declare_parameter("filter_alpha_acc", filter_alpha_acc_, desc_acc);

    // filter_alpha_gyro 슬라이더
    rcl_interfaces::msg::ParameterDescriptor desc_gyro;
    desc_gyro.description = "자이로 로우패스 필터 알파값 (0.0~1.0)";
    rcl_interfaces::msg::FloatingPointRange range_gyro;
    range_gyro.from_value = 0.0;
    range_gyro.to_value = 1.0;
    range_gyro.step = 0.01;
    desc_gyro.floating_point_range.push_back(range_gyro);
    this->declare_parameter("filter_alpha_gyro", filter_alpha_gyro_, desc_gyro);

    // filter_alpha_gravity 슬라이더
    rcl_interfaces::msg::ParameterDescriptor desc_gravity;
    desc_gravity.description = "중력 로우패스 필터 알파값 (0.0~1.0)";
    rcl_interfaces::msg::FloatingPointRange range_gravity;
    range_gravity.from_value = 0.0;
    range_gravity.to_value = 1.0;
    range_gravity.step = 0.01;
    desc_gravity.floating_point_range.push_back(range_gravity);
    this->declare_parameter("filter_alpha_gravity", filter_alpha_gravity_, desc_gravity);

    // filter_alpha_quat 슬라이더
    rcl_interfaces::msg::ParameterDescriptor desc_quat;
    desc_quat.description = "쿼터니언 로우패스 필터 알파값 (0.0~1.0)";
    rcl_interfaces::msg::FloatingPointRange range_quat;
    range_quat.from_value = 0.0;
    range_quat.to_value = 1.0;
    range_quat.step = 0.01;
    desc_quat.floating_point_range.push_back(range_quat);
    this->declare_parameter("filter_alpha_quat", filter_alpha_quat_, desc_quat);

    // 파라미터 변경 콜백 등록
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ImuSerialReceiver::onParameterChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "필터 알파 슬라이더 파라미터 설정 완료");
}

rcl_interfaces::msg::SetParametersResult ImuSerialReceiver::onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        if (param.get_name() == "filter_alpha_acc")
        {
            filter_alpha_acc_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "filter_alpha_acc 변경: %f", filter_alpha_acc_);
        }
        else if (param.get_name() == "filter_alpha_gyro")
        {
            filter_alpha_gyro_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "filter_alpha_gyro 변경: %f", filter_alpha_gyro_);
        }
        else if (param.get_name() == "filter_alpha_gravity")
        {
            filter_alpha_gravity_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "filter_alpha_gravity 변경: %f", filter_alpha_gravity_);
        }
        else if (param.get_name() == "filter_alpha_quat")
        {
            filter_alpha_quat_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "filter_alpha_quat 변경: %f", filter_alpha_quat_);
        }
    }

    return result;
}
