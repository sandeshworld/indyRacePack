import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

#topic used to listen to the lidar message
from 

#topic used to control vehicle - make sure to install these messages to use them
from lgsvl_msgs.msg import VehicleControlData

# this class will subscribe to the lidar topic and then publish
# to the vehicle control publisher to maintain follow following going around the track


class VehicleControlPublisher(Node):

    def __init__(self):
        super().__init__('wall_following')
        self.publisher_ = self.create_publisher(VehicleControlData, '/lgsvl/vehicle_control_cmd', 10)
        # timer_period = 0.5  # seconds

        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar_front/points_raw',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning

        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # def timer_callback(self):
    #     msg = VehicleControlData()
    #     msg.acceleration_pct = 0.1*0.1*self.i
    #     msg.braking_pct = 0.0
    #     msg.target_gear = 1
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.acceleration_pct)
    #     self.i += 1

    def returnDistance(self, msg):
        msg.data

    def listener_callback(self, data):

        distanceFromWall = self.returnDistance(data)
        
        msg = VehicleControlData()
        msg.acceleration_pct = 0.1*0.1*self.i
        msg.braking_pct = 0.0
        msg.target_gear = 1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.acceleration_pct)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    vehicleControlPublisher = VehicleControlPublisher()

    rclpy.spin(vehicleControlPublisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicleControlPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
