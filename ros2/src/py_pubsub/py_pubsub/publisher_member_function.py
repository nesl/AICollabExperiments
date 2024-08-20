import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner

from sensor_interfaces.msg import RSSI                            # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(RSSI, 'rssi', 10)  # CHANGE
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            ble_list = Scanner().scan(1.0)
            for dev in ble_list:
                if dev.getValueText(9):
                    msg = RSSI()                                                # CHANGE
                    msg.num = dev.rssi
                    msg.name = dev.getValueText(9)
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: %s "%d"' % (msg.name,msg.num))       # CHANGE

        except:
            raise Exception("Error ocurred")



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
