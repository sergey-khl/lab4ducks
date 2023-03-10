#!/usr/bin/env python3

import rospy
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern, ChangePatternResponse
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String

# References:   https://github.com/duckietown/dt-core/blob/6d8e99a5849737f86cab72b04fd2b449528226be/packages/led_emitter/src/led_emitter_node.py#L254
#               https://github.com/anna-ssi/mobile-robotics/blob/50d0b24eab13eb32d92fa83273a05564ca4dd8ef/assignment2/src/led_node.py

class LEDNode(DTROS):
    def __init__(self, node_name: str) -> None:
        '''
        +------------------+------------------------------------------+
        | Index            | Position (rel. to direction of movement) |
        +==================+==========================================+
        | 0                | Front left                               |
        +------------------+------------------------------------------+
        | 1                | Rear left                                |
        +------------------+------------------------------------------+
        | 2                | Top / Front middle                       |
        +------------------+------------------------------------------+
        | 3                | Rear right                               |
        +------------------+------------------------------------------+
        | 4                | Front right                              |
        +------------------+------------------------------------------+
        '''
        super(LEDNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")

        # -- Proxies -- 
        self.setCustomPattern = rospy.ServiceProxy(
            f'/{self.veh_name}/led_emitter_node/set_custom_pattern',
            SetCustomLEDPattern
        )

        # -- Publishers --
        self.pub_leds = rospy.Publisher(
            "~led_pattern",
            LEDPattern, 
            queue_size=1, 
            dt_topic_type=TopicType.DRIVER
        )

        # -- Servers -- 
        self.server = rospy.Service(
            f'/{self.veh_name}/led_controller_node/led_pattern', 
            ChangePattern, 
            self.handle_change_led_msg
        )

        self.colors = {
            "off": [0, 0, 0],
            "white": [1, 1, 1],
            "green": [0, 1, 0],
            "red": [1, 0, 0],
            "blue": [0, 0, 1],
            "yellow": [1, 0.8, 0],
            "purple": [1, 0, 1],
            "cyan": [0, 1, 1],
            "pink": [1, 0, 0.5],
        }

    def handle_change_led_msg(self, msg: ChangePattern):
        '''
        Changing the led msg to the one we want to use.
        '''
        new_msg = LEDPattern()

        new_msg.color_list = [msg.pattern_name.data] * 5
        new_msg.color_mask = [1, 1, 1, 1, 1]
        new_msg.frequency = 0.0
        new_msg.frequency_mask = [0, 0, 0, 0, 0]
        self.setCustomPattern(new_msg)

        return ChangePatternResponse()


if __name__ == "__main__":
    node = LEDNode(node_name="led_node")
    rospy.spin()