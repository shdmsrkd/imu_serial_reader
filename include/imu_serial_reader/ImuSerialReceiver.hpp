#ifndef SERIAL_IMU_RECEIVER_HPP
#define SERIAL_IMU_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <shared_mutex>
#include <atomic>

class ImuSerialReceiver : public rclcpp::Node
{
  public:
    ImuSerialReceiver(const std::string &port = "/dev/bno_imu", unsigned int baud_rate = 460800);
    ~ImuSerialReceiver();

    void openPort();
    void closePort();
    void readLoop();

    double getFilterAlphaAcc() const { return filter_alpha_acc_; }
    double getFilterAlphaGyro() const { return filter_alpha_gyro_; }
    double getFilterAlphaGravity() const { return filter_alpha_gravity_; }
    double getFilterAlphaQuat() const { return filter_alpha_quat_; }
    const std::string& getTopicNameImuData() const { return topic_name_imu_data_; }
    const std::string& getTopicNameGravity() const { return topic_name_gravity_; }
    const std::string& getFrameId() const { return frame_id_; }
    bool isDataReady() const { return data_ready_.load(); }
    void consumeData() { data_ready_.store(false); }

    void initParameters();
    void setupSliderParameter();
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & parameters);
    #pragma pack(push, 1)
    typedef struct {
        uint8_t header[2];
        float rpy[3];
        float quat[4];
        float acc[3];
        float gyro[3];
        float gravity[3];
        uint8_t checksum;
    } IMU_Packet_t;
    #pragma pack(pop)

    IMU_Packet_t sensor_data;
    mutable std::shared_mutex sensor_mtx_;

  private:
    bool parseFrame(const std::vector<uint8_t>& packet, IMU_Packet_t& data_out);

    std::string port_name_;
    unsigned int baud_rate_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::thread read_thread;
    bool running;

    std::vector<uint8_t> data_buffer_;
    std::atomic<bool> data_ready_{false};

    std::string device_port_;
    std::string frame_id_;
    std::string topic_name_imu_data_;
    std::string topic_name_gravity_;
    double filter_alpha_acc_;
    double filter_alpha_gyro_;
    double filter_alpha_gravity_;
    double filter_alpha_quat_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif
