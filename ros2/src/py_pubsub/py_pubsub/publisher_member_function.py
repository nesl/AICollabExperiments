import rclpy
from datetime import datetime
from pathlib import Path
import pydbus
from rclpy.node import Node

from sensor_interfaces.msg import RSSI                            # CHANGE

bus = pydbus.SystemBus()

class DeviceMonitor:
    """Class to represent remote bluetooth devices discovered"""
    def __init__(self, path_obj, pub):
        self.device = bus.get('org.bluez', path_obj)
        self.device.onPropertiesChanged = self.prop_changed
        rssi = self.device.GetAll('org.bluez.Device1').get('RSSI')
        self.name = self.device.GetAll('org.bluez.Device1').get('Name')
        self.pub = pub

        if rssi is not None and self.name:
            msg = RSSI()
            msg.num = rssi
            msg.name = self.name
            self.pub.publisher_.publish(msg)
            self.pub.get_logger().info('Publishing RSSI: %s %d' % (self.name,rssi))       # CHANGE

        #if rssi and self.name:
        #    print(f'Device added to monitor {self.name} {self.device.Address} @ {rssi} dBm')
        #else:
        #    print(f'Device added to monitor {self.device.Address}')

    def prop_changed(self, iface, props_changed, props_removed):
        """method to be called when a property value on a device changes"""
        rssi = props_changed.get('RSSI', None)

        if rssi is not None and self.name:
            msg = RSSI()
            msg.num = rssi
            msg.name = self.name
            self.pub.publisher_.publish(msg)
            self.pub.get_logger().info('Publishing: "%d"' % msg.num)       # CHANGE

            #print(f'\tDevice Seen: {self.name} {self.device.Address} @ {rssi} dBm')
            #write_to_log(self.device.Address, rssi)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(RSSI, 'rssi', 10)  # CHANGE
 
        # BlueZ object manager
        mngr = bus.get('org.bluez', '/')
        mngr.onInterfacesAdded = self.new_iface


        # Iterate around already known devices and add to monitor
        mng_objs = mngr.GetManagedObjects()
        for path in mng_objs:
            device = mng_objs[path].get('org.bluez.Device1', {}).get('Address', [])
            if device:
                DeviceMonitor(path, self)

    def new_iface(self, path, iface_props):
        """If a new dbus interfaces is a device, add it to be  monitored"""
        device_addr = iface_props.get('org.bluez.Device1', {}).get('Address')
        if device_addr:
            DeviceMonitor(path, self)


def main(args=None):
    rclpy.init(args=args)


    # Connect to the DBus api for the Bluetooth adapter
    adapter = bus.get('org.bluez', '/org/bluez/hci0')
    adapter.DuplicateData = False


    # Run discovery for discovery_time
    adapter.StartDiscovery()


    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    adapter.StopDiscovery()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
