import rclpy
from rclpy.node import Node
import carla
import time

import yaml

# Import custumed msg, srv from autoware ws
from autoware_auto_perception_msgs.msg import TrafficSignal, TrafficSignalArray, TrafficLight


class TrafficLightBridge(Node):
    def __init__(self):
        super().__init__('traffic_light_bridge')
        self.get_logger().info('Traffic_light_bridge Initilized')

        self.traffic_light_topic_ = "/perception/traffic_light_recognition/traffic_signals"

        self.declare_parameters(
            namespace='',
            parameters=[
                ('carla_host', rclpy.Parameter.Type.STRING),
                ('carla_port', rclpy.Parameter.Type.INTEGER),
                ('carla_traffic_light_topic', rclpy.Parameter.Type.STRING),
                ('pub_all', rclpy.Parameter.Type.BOOL),
                ('cl_aw_signal_mapping_file', rclpy.Parameter.Type.STRING)
            ]
        )

        # Connect to CARLA
        self.carla_host_ = self.get_parameter('carla_host').get_parameter_value().string_value
        self.carla_port_ = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.client_ = carla.Client(self.carla_host_, self.carla_port_)
        self.client_.set_timeout(5.0)
        self.world_ = self.client_.get_world()

        # Whether publish all traffic light or only the one affecting ego_vehicle
        self.pub_all_ = self.get_parameter('pub_all').get_parameter_value().bool_value

        # Create a map of traffic light id in carla to traffic light id in autoware
        cl_aw_signal_mapping_file = self.get_parameter('cl_aw_signal_mapping_file').get_parameter_value().string_value

        with open(cl_aw_signal_mapping_file, 'r') as file:
            self.cl_aw_signal_map_ = yaml.safe_load(file)

        # sleep for 1 second to wait for the carla to tick
        time.sleep(1)
        # Find the ego_vheicle
        self.agent_role_name_ = "hero"
        self.ego_vehicle_ = self.get_agent_actor(self.world_, self.agent_role_name_)

        self.traffic_light_pub_ = self.create_publisher(
            TrafficSignalArray,
            self.traffic_light_topic_,
            1)
        
        # Print the traffic_light_id afftected by ego_vehicle in loop

        if self.pub_all_:
            self.timer_ = self.create_timer(0.5, self.pub_traffic_light_all)
        else:
            self.timer_ = self.create_timer(0.5, self.pub_traffic_light_affect)
        
        

    # This function publish all traffic on the set route of the ego_vehicle 
    def pub_traffic_light_all(self):
        traffic_signal_array = TrafficSignalArray()
        for carla_id, aw_id in self.cl_aw_signal_map_.items():
            carla_traffic_signal_ = self. world_.get_traffic_light_from_opendrive_id(carla_id)
            traffic_signal_ = self.get_signal(carla_traffic_signal_)
            traffic_signal_array.signals.append(traffic_signal_)
        
        traffic_signal_array.header.frame_id = "map"
        traffic_signal_array.header.stamp = self.get_clock().now().to_msg()
        self.traffic_light_pub_.publish(traffic_signal_array)

    # This function publish the traffic light affecting ego_vehicle
    def pub_traffic_light_affect(self):
        if (self.ego_vehicle_.get_traffic_light() is not None):
            self.get_logger().info("ego_vehicle_traffic_light_id: %s" % self.ego_vehicle_.get_traffic_light().get_opendrive_id())
            carla_traffic_light = self.ego_vehicle_.get_traffic_light()

            traffic_signal = self.get_signal(carla_traffic_light)
            # Build a sginal array
            traffic_signal_array = TrafficSignalArray()
            traffic_signal_array.header.frame_id = "map"
            traffic_signal_array.header.stamp = self.get_clock().now().to_msg()
            traffic_signal_array.signals.append(traffic_signal)
            
            self.traffic_light_pub_.publish(traffic_signal_array)

    # Define helper function to get agent actor
    def get_agent_actor(self, world, role_name):
            actors = None
            actors = world.get_actors()
            for car in actors:
                if car.attributes['role_name'] == role_name:
                    return car
            return None
    
    # Define helper function to get traffic signal
    def get_signal(self, carla_traffic_light):
        carla_signal_color = carla_traffic_light.get_state()
        carla_light_id = carla_traffic_light.get_opendrive_id()

        # Build a traffic_light
        traffic_light = self.get_light(carla_signal_color)

        # Build a traffic signal
        traffic_signal = TrafficSignal()
        aw_light_id = self.cl_aw_signal_map_[carla_light_id]
        traffic_signal.map_primitive_id = int(aw_light_id)
        traffic_signal.lights.append(traffic_light)

        return traffic_signal

    # Define helper function to get traffic light
    def get_light(self, signal_color):
        traffic_light = TrafficLight()
        traffic_light.confidence = 1.0
        traffic_light.shape = TrafficLight.CIRCLE
        traffic_light.status = TrafficLight.SOLID_ON
        # Determine light color
        if (signal_color == carla.TrafficLightState.Red):
            traffic_light.color  = TrafficLight.RED
        elif (signal_color == carla.TrafficLightState.Yellow):
            traffic_light.color  = TrafficLight.AMBER
        elif (signal_color == carla.TrafficLightState.Green):
            traffic_light.color = TrafficLight.GREEN

        return traffic_light


def main():
    rclpy.init()
    node = TrafficLightBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

