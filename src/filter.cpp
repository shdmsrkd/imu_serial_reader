#include "imu_serial_reader/filter.hpp"

Filter::Filter(float alpha) : alpha_(alpha), initialized_(false), initialized4_(false), buffer_index_(0), buffer_count_(0)
{
    filtered_value_[0] = 0.0f;
    filtered_value_[1] = 0.0f;
    filtered_value_[2] = 0.0f;
    filtered_value4_[0] = 0.0f;
    filtered_value4_[1] = 0.0f;
    filtered_value4_[2] = 0.0f;
    filtered_value4_[3] = 0.0f;

    // 100개 버퍼 초기화 (3-value buffer)
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer_3_[i][0] = 0.0f;
        buffer_3_[i][1] = 0.0f;
        buffer_3_[i][2] = 0.0f;
    }

    // 100개 버퍼 초기화 (4-value buffer)
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer_4_[i][0] = 0.0f;
        buffer_4_[i][1] = 0.0f;
        buffer_4_[i][2] = 0.0f;
        buffer_4_[i][3] = 0.0f;
    }
}

void Filter::lowPassFilterUpdate(const float input[3])
{
    // 새로운 값을 버퍼에 저장 (순환 버퍼)
    buffer_3_[buffer_index_][0] = input[0];
    buffer_3_[buffer_index_][1] = input[1];
    buffer_3_[buffer_index_][2] = input[2];

    // 버퍼 인덱스와 카운트 업데이트
    buffer_index_ = (buffer_index_ + 1) % BUFFER_SIZE;
    if (buffer_count_ < BUFFER_SIZE) {
        buffer_count_++;
    }

    // 버퍼의 모든 값들에 LPF 적용
    float avg[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < buffer_count_; i++) {
        avg[0] += buffer_3_[i][0];
        avg[1] += buffer_3_[i][1];
        avg[2] += buffer_3_[i][2];
    }
    avg[0] /= buffer_count_;
    avg[1] /= buffer_count_;
    avg[2] /= buffer_count_;

    // 저패스 필터 적용 (알파값 사용)
    if (!initialized_)
    {
        filtered_value_[0] = avg[0];
        filtered_value_[1] = avg[1];
        filtered_value_[2] = avg[2];
        initialized_ = true;
    }
    else
    {
        filtered_value_[0] = alpha_ * avg[0] + (1.0f - alpha_) * filtered_value_[0];
        filtered_value_[1] = alpha_ * avg[1] + (1.0f - alpha_) * filtered_value_[1];
        filtered_value_[2] = alpha_ * avg[2] + (1.0f - alpha_) * filtered_value_[2];
    }
}

void Filter::getFiltered(float output[3]) const
{
    output[0] = filtered_value_[0];
    output[1] = filtered_value_[1];
    output[2] = filtered_value_[2];
}

void Filter::setAlpha(float alpha)
{
    alpha_ = alpha;
}

void Filter::lowPassFilterUpdate4(const float input[4])
{
    // 새로운 값을 버퍼에 저장 (순환 버퍼)
    buffer_4_[buffer_index_][0] = input[0];
    buffer_4_[buffer_index_][1] = input[1];
    buffer_4_[buffer_index_][2] = input[2];
    buffer_4_[buffer_index_][3] = input[3];

    // 버퍼가 가득 찬 후부터는 buffer_count_가 BUFFER_SIZE를 유지
    if (buffer_count_ < BUFFER_SIZE) {
        buffer_count_++;
    }

    // 버퍼의 모든 값들에 LPF 적용
    float avg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < buffer_count_; i++) {
        avg[0] += buffer_4_[i][0];
        avg[1] += buffer_4_[i][1];
        avg[2] += buffer_4_[i][2];
        avg[3] += buffer_4_[i][3];
    }
    avg[0] /= buffer_count_;
    avg[1] /= buffer_count_;
    avg[2] /= buffer_count_;
    avg[3] /= buffer_count_;

    // 저패스 필터 적용 (알파값 사용)
    if (!initialized4_)
    {
        filtered_value4_[0] = avg[0];
        filtered_value4_[1] = avg[1];
        filtered_value4_[2] = avg[2];
        filtered_value4_[3] = avg[3];
        initialized4_ = true;
    }
    else
    {
        filtered_value4_[0] = alpha_ * avg[0] + (1.0f - alpha_) * filtered_value4_[0];
        filtered_value4_[1] = alpha_ * avg[1] + (1.0f - alpha_) * filtered_value4_[1];
        filtered_value4_[2] = alpha_ * avg[2] + (1.0f - alpha_) * filtered_value4_[2];
        filtered_value4_[3] = alpha_ * avg[3] + (1.0f - alpha_) * filtered_value4_[3];
    }
}

void Filter::getFiltered4(float output[4]) const
{
    output[0] = filtered_value4_[0];
    output[1] = filtered_value4_[1];
    output[2] = filtered_value4_[2];
    output[3] = filtered_value4_[3];
}
