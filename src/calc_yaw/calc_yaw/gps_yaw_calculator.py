import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import math

class GpsYawCalculator(Node):
    """
    두 개의 GPS 센서로부터 데이터를 받아 Global Yaw (Heading)를 계산하는 노드.
    
    - 두 GPS 토픽을 구독 (기본: /f9p/fix, /f9r/fix).
    - 두 메시지의 타임스탬프를 비교하여 동기화.
    - 위도/경도를 사용하여 두 지점 간의 방위각(bearing)을 계산.
    - 계산된 Yaw를 /global_yaw 토픽으로 발행.
    """
    def __init__(self):
        super().__init__('gps_yaw_calculator')

        # 파라미터 선언 (토픽 이름, 동기화 시간 임계값 등)
        self.declare_parameter('gps1_topic', '/f9r/fix') #차량 뒤쪽
        self.declare_parameter('gps2_topic', '/f9p/fix') #차량 앞쪽
        self.declare_parameter('yaw_topic', '/global_yaw')
        self.declare_parameter('max_time_diff_sec', 0.1)

        # 파라미터 값 가져오기
        gps1_topic = self.get_parameter('gps1_topic').get_parameter_value().string_value
        gps2_topic = self.get_parameter('gps2_topic').get_parameter_value().string_value
        yaw_topic = self.get_parameter('yaw_topic').get_parameter_value().string_value
        self.max_time_diff = self.get_parameter('max_time_diff_sec').get_parameter_value().double_value

        # 마지막으로 수신한 GPS 메시지를 저장할 변수
        self.gps1_fix = None
        self.gps2_fix = None

        # GPS 토픽 구독자 설정
        self.gps1_subscription = self.create_subscription(
            NavSatFix,
            gps1_topic,
            self.gps1_callback,
            10)
        
        self.gps2_subscription = self.create_subscription(
            NavSatFix,
            gps2_topic,
            self.gps2_callback,
            10)

        # 계산된 Yaw를 발행할 Publisher 설정
        self.yaw_publisher = self.create_publisher(Float64, yaw_topic, 10)

        self.get_logger().info(f"GPS Yaw Calculator 노드가 시작되었습니다.")
        self.get_logger().info(f"  - GPS 1 Topic: {gps1_topic}")
        self.get_logger().info(f"  - GPS 2 Topic: {gps2_topic}")
        self.get_logger().info(f"  - Yaw Topic: {yaw_topic}")

    def gps1_callback(self, msg):
        """GPS1 토픽의 콜백 함수"""
        self.gps1_fix = msg
        self.try_calculate_yaw()

    def gps2_callback(self, msg):
        """GPS2 토픽의 콜백 함수"""
        self.gps2_fix = msg
        self.try_calculate_yaw()

    def try_calculate_yaw(self):
        """두 GPS 데이터가 모두 유효할 때 Yaw 계산을 시도"""
        if self.gps1_fix is None or self.gps2_fix is None:
            return

        # 시간 동기화 확인
        time1 = self.gps1_fix.header.stamp.sec + self.gps1_fix.header.stamp.nanosec / 1e9
        time2 = self.gps2_fix.header.stamp.sec + self.gps2_fix.header.stamp.nanosec / 1e9
        
        if abs(time1 - time2) > self.max_time_diff:
            self.get_logger().warn(
                f"GPS 타임스탬프 차이가 너무 큽니다: {abs(time1 - time2):.4f}s. Yaw 계산을 건너뜁니다."
            )
            return

        # 위도와 경도를 라디안으로 변환
        lat1 = math.radians(self.gps1_fix.latitude)
        lon1 = math.radians(self.gps1_fix.longitude)
        lat2 = math.radians(self.gps2_fix.latitude)
        lon2 = math.radians(self.gps2_fix.longitude)

        # 방위각(Bearing) 계산
        # GPS1을 기준점으로 GPS2의 방향을 계산합니다. 
        # f9r이 뒤쪽, f9p가 앞쪽이므로 f9r -> f9p 방향이 차량의 진행 방향
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        bearing_rad = math.atan2(y, x)

        # 라디안을 도로 변환하고 0-360도 범위로 정규화
        bearing_deg = (math.degrees(bearing_rad) + 360) % 360

        # 결과 발행
        yaw_msg = Float64()
        yaw_msg.data = bearing_deg
        self.yaw_publisher.publish(yaw_msg)

        self.get_logger().info(f"계산된 Global Yaw: {bearing_deg:.2f}°")
        
        # 한 번 계산에 사용된 데이터는 초기화하여 중복 계산 방지
        self.gps1_fix = None
        self.gps2_fix = None


def main(args=None):
    rclpy.init(args=args)
    node = GpsYawCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
