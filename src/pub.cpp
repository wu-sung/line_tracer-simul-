#include "ltlt/pub.hpp"

Pub::Pub() : Node("campub")
{
    // QoS 설정 - 마지막 10개의 메시지를 유지
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 압축된 이미지를 게시하는 퍼블리셔 생성
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    
    // 25ms마다 메시지를 게시하기 위한 타이머 설정
    timer_ = this->create_wall_timer(25ms, std::bind(&Pub::publish_msg, this));
    
    // 비디오 캡처를 위한 GStreamer 파이프라인 초기화
    if (!cap.open(src, cv::CAP_GSTREAMER)) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video using GStreamer pipeline!");
        rclcpp::shutdown();
        return;
    }

    // Dynamixel 모터 초기화
    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return;
    }

    // err 주제를 구독하고 콜백 함수 등록
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", qos_profile, fn);
}

// 메시지 게시 함수
void Pub::publish_msg()
{
    // 비디오 캡처에서 프레임을 읽음
    cap >> frame;
    if (frame.empty()) { 
        RCLCPP_ERROR(this->get_logger(), "frame empty"); 
        return;
    }
    
    // 프레임을 압축된 이미지 메시지로 변환하여 퍼블리셔를 통해 게시
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

// 구독 콜백 함수
void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    // 수신된 에러 값에 따라 모터 속도 조정
    err = intmsg->data;
    lvel = 100 - gain * err; // 왼쪽 바퀴 속도
    rvel = -(100 + gain * err); // 오른쪽 바퀴 속도
    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    
    // Dynamixel 모터의 속도 설정
    mdxl.setVelocity(lvel, rvel);
}
