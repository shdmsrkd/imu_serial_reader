#include "imu_serial_reader/stateMachine.hpp"
#include <cmath>

namespace {
void multiplyQuaternion(double ax, double ay, double az, double aw,
                        double bx, double by, double bz, double bw,
                        double& rx, double& ry, double& rz, double& rw)
{
    rx = aw * bx + ax * bw + ay * bz - az * by;
    ry = aw * by - ax * bz + ay * bw + az * bx;
    rz = aw * bz + ax * by - ay * bx + az * bw;
    rw = aw * bw - ax * bx - ay * by - az * bz;
}

void normalizeQuaternion(double& x, double& y, double& z, double& w)
{
    const double norm = std::sqrt(x*x + y*y + z*z + w*w);
    if (norm > 1e-12)
    {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }
    else
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    }
}
}

StateMachine::StateMachine(ImuSerialReceiver* imu_receiver, Filter* acc_filter, Filter* gyro_filter, Filter* gravity_filter, Filter* quat_filter, Filter* rpy_filter)
    : imu_receiver_handler_(imu_receiver),
      acc_filter_(acc_filter),
      gyro_filter_(gyro_filter),
      gravity_filter_(gravity_filter),
      quat_filter_(quat_filter),
            rpy_filter_(rpy_filter),
            has_valid_sample_(false),
      current_state(State::READING)
{
    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // 퍼블리셔 생성
    imu_pub_ = imu_receiver->create_publisher<sensor_msgs::msg::Imu>(imu_receiver->getTopicNameImuData(), qos_profile);
    gravity_pub_ = imu_receiver->create_publisher<geometry_msgs::msg::Vector3Stamped>(imu_receiver->getTopicNameGravity(), qos_profile);
    rpy_pub_ = imu_receiver->create_publisher<geometry_msgs::msg::Vector3Stamped>(imu_receiver->getTopicNameRpy(), qos_profile);
    gravity_marker_pub_ = imu_receiver->create_publisher<visualization_msgs::msg::Marker>("imu/gravity_marker", qos_profile);
}

StateMachine::~StateMachine()
{
}

void StateMachine::setState(State new_state)
{
    current_state = new_state;
}

StateMachine::State StateMachine::getState() const
{
    return current_state;
}

bool StateMachine::StateControl()
{
    if (!imu_receiver_handler_ || !acc_filter_ || !gyro_filter_ || !gravity_filter_ || !rpy_filter_)
    {
        return false;
    }

    try
    {
        current_state = State::READING;

        if (imu_receiver_handler_->isDataReady())
        {
            current_state = State::FILTERING;
            LPFSlider(
                imu_receiver_handler_->getFilterAlphaAcc(),
                imu_receiver_handler_->getFilterAlphaGyro(),
                imu_receiver_handler_->getFilterAlphaGravity(),
                imu_receiver_handler_->getFilterAlphaQuat(),
                imu_receiver_handler_->getFilterAlphaRpy());
            Filtering();
            imu_receiver_handler_->consumeData();
            has_valid_sample_ = true;
        }

        if (!has_valid_sample_)
        {
            return true;
        }

        current_state = State::PUBLISHING;
        Publishing();
        current_state = State::READING;
    }
    catch (const std::system_error& e)
    {
        RCLCPP_ERROR(imu_receiver_handler_->get_logger(),
                     "System error in state %d: %s (code: %d)",
                     static_cast<int>(current_state), e.what(), e.code().value());
        error_count_++;
        current_state = State::ERROR;
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(imu_receiver_handler_->get_logger(),
                     "Exception in state %d: %s",
                     static_cast<int>(current_state), e.what());
        error_count_++;
        current_state = State::ERROR;
        return false;
    }
    catch (...)
    {
        RCLCPP_ERROR(imu_receiver_handler_->get_logger(),
                     "Unknown error in state %d",
                     static_cast<int>(current_state));
        error_count_++;
        current_state = State::ERROR;
        return false;
    }

    return true;
}

void StateMachine::Filtering()
{
        // 가속도 필터링
        acc_filter_->lowPassFilterUpdate(imu_receiver_handler_->sensor_data.acc);
        // 자이로 필터링
        gyro_filter_->lowPassFilterUpdate(imu_receiver_handler_->sensor_data.gyro);
        // 중력 벡터 필터링
        gravity_filter_->lowPassFilterUpdate(imu_receiver_handler_->sensor_data.gravity);
        // 쿼터니언 필터링
        quat_filter_->lowPassFilterUpdate4(imu_receiver_handler_->sensor_data.quat);
        // RPY 필터링
        rpy_filter_->lowPassFilterUpdate(imu_receiver_handler_->sensor_data.rpy);
}

void StateMachine::FilterUpdating()
{
        float filtered_acc[3];
        float filtered_gyro[3];
        float filtered_gravity[3];
        float filtered_quat[4];
        float filtered_rpy[3];

        acc_filter_->getFiltered(filtered_acc);
        gyro_filter_->getFiltered(filtered_gyro);
        gravity_filter_->getFiltered(filtered_gravity);
        quat_filter_->getFiltered4(filtered_quat);
        rpy_filter_->getFiltered(filtered_rpy);

        std::unique_lock<std::shared_mutex> lk(imu_receiver_handler_->sensor_mtx_);
        // 가속도 업데이트
        imu_receiver_handler_->sensor_data.acc[0] = filtered_acc[0];
        imu_receiver_handler_->sensor_data.acc[1] = filtered_acc[1];
        imu_receiver_handler_->sensor_data.acc[2] = filtered_acc[2];
        // 자이로 업데이트
        imu_receiver_handler_->sensor_data.gyro[0] = filtered_gyro[0];
        imu_receiver_handler_->sensor_data.gyro[1] = filtered_gyro[1];
        imu_receiver_handler_->sensor_data.gyro[2] = filtered_gyro[2];
        // 중력 벡터 업데이트
        imu_receiver_handler_->sensor_data.gravity[0] = filtered_gravity[0];
        imu_receiver_handler_->sensor_data.gravity[1] = filtered_gravity[1];
        imu_receiver_handler_->sensor_data.gravity[2] = filtered_gravity[2];
        // 쿼터니언 업데이트
        imu_receiver_handler_->sensor_data.quat[0] = filtered_quat[0];
        imu_receiver_handler_->sensor_data.quat[1] = filtered_quat[1];
        imu_receiver_handler_->sensor_data.quat[2] = filtered_quat[2];
        imu_receiver_handler_->sensor_data.quat[3] = filtered_quat[3];
        // RPY 업데이트
        imu_receiver_handler_->sensor_data.rpy[0] = filtered_rpy[0];
        imu_receiver_handler_->sensor_data.rpy[1] = filtered_rpy[1];
        imu_receiver_handler_->sensor_data.rpy[2] = filtered_rpy[2];
}

void StateMachine::GravityMarkerPublishing()
{
    // 센서 데이터 로컬 복사
    ImuSerialReceiver::IMU_Packet_t local{};
    {
        std::shared_lock<std::shared_mutex> lk(imu_receiver_handler_->sensor_mtx_);
        local = imu_receiver_handler_->sensor_data;
    }

    // 쿼터니언 (센서 → 월드 회전)
    double qx = local.quat[0];
    double qy = local.quat[1];
    double qz = local.quat[2];
    double qw = local.quat[3];

    // 센서 좌표계의 중력 벡터
    double gx = -local.gravity[0];
    double gy = -local.gravity[1];
    double gz = -local.gravity[2];

    // 쿼터니언으로 벡터를 월드 좌표계로 변환: v' = q * v * q^(-1)
    // 회전 행렬 전개
    double wx = (1.0 - 2.0*(qy*qy + qz*qz)) * gx + 2.0*(qx*qy - qz*qw) * gy + 2.0*(qx*qz + qy*qw) * gz;
    double wy = 2.0*(qx*qy + qz*qw) * gx + (1.0 - 2.0*(qx*qx + qz*qz)) * gy + 2.0*(qy*qz - qx*qw) * gz;
    double wz = 2.0*(qx*qz - qy*qw) * gx + 2.0*(qy*qz + qx*qw) * gy + (1.0 - 2.0*(qx*qx + qy*qy)) * gz;

    // 크기 정규화 (단위 벡터로 만들어 일정한 길이의 화살표 표시)
    double norm = std::sqrt(wx*wx + wy*wy + wz*wz);
    if (norm > 0.01)
    {
        wx /= norm;
        wy /= norm;
        wz /= norm;
    }

    // 중력 벡터 화살표 마커
    auto marker_msg = visualization_msgs::msg::Marker();
    marker_msg.header.stamp = imu_receiver_handler_->now();
    marker_msg.header.frame_id = imu_receiver_handler_->getFrameId();
    marker_msg.ns = "gravity_vector";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;

    // pose.orientation = identity (기본값, 변환하지 않음)
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

    // 화살표 시작점 (원점)
    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    // 화살표 끝점 (월드 좌표계 기준 중력 방향, imu_link 프레임 내에서
    // 역회전된 좌표이므로 rviz에서 imu_link가 회전해도 상쇄됨)
    geometry_msgs::msg::Point end;
    end.x = wx;
    end.y = wy;
    end.z = wz;

    marker_msg.points.push_back(start);
    marker_msg.points.push_back(end);

    // 화살표 크기 (shaft diameter, head diameter, head length)
    marker_msg.scale.x = 0.05;  // shaft diameter
    marker_msg.scale.y = 0.1;   // head diameter
    marker_msg.scale.z = 0.1;   // head length

    // 색상 (초록색)
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    marker_msg.lifetime = rclcpp::Duration(0, 0);  // 영구 표시

    gravity_marker_pub_->publish(marker_msg);
}

void StateMachine::LPFSlider(double alpha_acc, double alpha_gyro, double alpha_gravity, double alpha_quat, double alpha_rpy)
{
    acc_filter_->setAlpha(alpha_acc);
    gyro_filter_->setAlpha(alpha_gyro);
    gravity_filter_->setAlpha(alpha_gravity);
    quat_filter_->setAlpha(alpha_quat);
    rpy_filter_->setAlpha(alpha_rpy);
}

void StateMachine::Publishing()
{
    // 필터링된 데이터 업데이트
    FilterUpdating();

    // ROS2 메시지 생성 및 발행
    ImuSerialReceiver::IMU_Packet_t local{};
    {
        std::shared_lock<std::shared_mutex> lk(imu_receiver_handler_->sensor_mtx_);
        local = imu_receiver_handler_->sensor_data;
    }

    double corrected_quat[4] = {
        static_cast<double>(local.quat[0]),
        static_cast<double>(local.quat[1]),
        static_cast<double>(local.quat[2]),
        static_cast<double>(local.quat[3])
    };
    double corrected_rpy[3] = {
        static_cast<double>(local.rpy[0]),
        static_cast<double>(local.rpy[1]),
        static_cast<double>(local.rpy[2])
    };

    if (imu_receiver_handler_->hasZeroReference())
    {
        double zero_quat_inv[4];
        double zero_rpy[3];
        imu_receiver_handler_->getZeroReference(zero_quat_inv, zero_rpy);

        multiplyQuaternion(
            zero_quat_inv[0], zero_quat_inv[1], zero_quat_inv[2], zero_quat_inv[3],
            corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3],
            corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3]);
        normalizeQuaternion(corrected_quat[0], corrected_quat[1], corrected_quat[2], corrected_quat[3]);

        corrected_rpy[0] -= zero_rpy[0];
        corrected_rpy[1] -= zero_rpy[1];
        corrected_rpy[2] -= zero_rpy[2];
    }

    // IMU 메시지
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = imu_receiver_handler_->now();
    imu_msg.header.frame_id = imu_receiver_handler_->getFrameId();

    imu_msg.orientation.x = corrected_quat[0];
    imu_msg.orientation.y = corrected_quat[1];
    imu_msg.orientation.z = corrected_quat[2];
    imu_msg.orientation.w = corrected_quat[3];

    imu_msg.angular_velocity.x = local.gyro[0];
    imu_msg.angular_velocity.y = local.gyro[1];
    imu_msg.angular_velocity.z = local.gyro[2];

    imu_msg.linear_acceleration.x = local.acc[0];
    imu_msg.linear_acceleration.y = local.acc[1];
    imu_msg.linear_acceleration.z = local.acc[2];

    imu_pub_->publish(imu_msg);

    // 중력 벡터 메시지
    auto gravity_msg = geometry_msgs::msg::Vector3Stamped();
    gravity_msg.header.stamp = imu_receiver_handler_->now();
    gravity_msg.header.frame_id = imu_receiver_handler_->getFrameId();
    gravity_msg.vector.x = local.gravity[0];
    gravity_msg.vector.y = local.gravity[1];
    gravity_msg.vector.z = local.gravity[2];

    gravity_pub_->publish(gravity_msg);

    // RPY 벡터 메시지
    auto rpy_msg = geometry_msgs::msg::Vector3Stamped();
    rpy_msg.header.stamp = imu_receiver_handler_->now();
    rpy_msg.header.frame_id = imu_receiver_handler_->getFrameId();
    rpy_msg.vector.x = corrected_rpy[0];
    rpy_msg.vector.y = corrected_rpy[1];
    rpy_msg.vector.z = corrected_rpy[2];

    rpy_pub_->publish(rpy_msg);

    // 중력 벡터 마커 발행
    GravityMarkerPublishing();
}

void StateMachine::HandleError()
{
    RCLCPP_WARN(imu_receiver_handler_->get_logger(), "In ERROR state. Attempting recovery...");
    imu_receiver_handler_->closePort();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    imu_receiver_handler_->openPort();
    current_state = State::READING;
    RCLCPP_INFO(imu_receiver_handler_->get_logger(), "Recovery attempt complete. Returning to READING state.");

    if (error_count_ >= 10)
    {
        RCLCPP_ERROR(imu_receiver_handler_->get_logger(), "Multiple recovery attempts failed. Manual intervention required.");
        imu_receiver_handler_->closePort();
        rclcpp::shutdown();
    }
}
