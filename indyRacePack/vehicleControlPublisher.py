import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

#topic used to control vehicle - make sure to install these messages to use them
from lgsvl_msgs.msg import VehicleControlData

#import from the other file
# from scanProcessor import *
import indyRacePack.scanProcessor as sp

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

        self.kp = 0.01
        self.kd = 0.005
        self.curr_error = 0.0
        self.curr_time = time.time()
        self.dist_setpoint = -13.0


    def listener_callback(self, data):
        #distanceFromWall = self.returnDistance(data)
        cloud_points = list(sp.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
        
        sum = 0
        count = 0
        for i in cloud_points:
            # print(i)
            if ((i[0] < 3) and (i[0] > 2)):
                if (i[1] < 0):
                    if ((i[2] < 1.5)):
                        sum += i[1]
                        count += 1
        
        dist = sum/count
        print("Dist: " + str(dist))

        # implementing pid loop for control for wall following -------------------
        prev_error = self.curr_error
        self.curr_error = dist - self.dist_setpoint
        print("Current Error: " + str(self.curr_error))
        
        # get delta t
        prev_time = self.curr_time
        self.cur_time = time.time()

        v0 = (self.kp*self.curr_error) + self.kd*(prev_error - self.curr_error)/(self.cur_time-prev_time)

        print("Correction: " + str(v0))


        # ---------------------------------


        msg = VehicleControlData()
        msg.acceleration_pct = 0.05 if -3 < v0 < 3 else 0.01
        msg.target_wheel_angle -= v0
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
