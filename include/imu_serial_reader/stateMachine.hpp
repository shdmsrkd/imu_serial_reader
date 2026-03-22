#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include "imu_serial_reader/ImuSerialReceiver.hpp"
#include "imu_serial_reader/filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class StateMachine
{
  public:
    StateMachine(ImuSerialReceiver* imu_receiver, Filter* acc_filter, Filter* gyro_filter, Filter* gravity_filter, Filter* quat_filter, Filter* rpy_filter);
    ~StateMachine();
    enum class State
    {
      READING,
      FILTERING,
      PUBLISHING,
      ERROR
    };

    bool StateControl();
    void setState(State new_state);
    State getState() const;
    void Filtering();
    void FilterUpdating();
    void Publishing();
    void HandleError();
    void GravityMarkerPublishing();
    void LPFSlider(double alpha_acc, double alpha_gyro, double alpha_gravity, double alpha_quat, double alpha_rpy);
  private:
    ImuSerialReceiver* imu_receiver_handler_;
    Filter* acc_filter_;
    Filter* gyro_filter_;
    Filter* gravity_filter_;
    Filter* quat_filter_;
    Filter* rpy_filter_;
    State current_state;
    int error_count_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gravity_marker_pub_;
};

#endif
