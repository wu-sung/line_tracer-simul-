#include "ltlt/pub.hpp"

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // Pub 클래스의 인스턴스를 생성하고, 공유 포인터로 관리
    auto node = std::make_shared<Pub>();

    // ROS2 스핀: 노드가 종료될 때까지 콜백 함수들을 실행
    rclcpp::spin(node);

    // ROS2 종료
    rclcpp::shutdown();

    // Dynamixel 객체 닫기
    node->dxl.close();

    return 0;
}
