#include "ltlt/sub.hpp"

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // Sub 클래스의 인스턴스를 생성하고, 이를 공유 포인터(std::shared_ptr)로 관리
    auto node = std::make_shared<Sub>();

    // ROS2 스핀: 노드가 종료될 때까지 콜백 함수들을 실행
    rclcpp::spin(node);

    // ROS2 종료
    rclcpp::shutdown();

    return 0;
}
