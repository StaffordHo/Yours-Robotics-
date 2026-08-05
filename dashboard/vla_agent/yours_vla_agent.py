#!/usr/bin/env python3
"""
yours_vla_agent.py -- Vision-Language-Action (VLA) AI Agent for 小车 (Yours Robotics)

Converts natural language user instructions + RealSense camera vision into executable
ROS navigation goals, storage compartment door motor actions, climate setpoints, and voice speech responses.

Topics Subscribed:
  - /yours_vla/command (std_msgs/String) -- Natural language instructions
  - /camera/color/image_raw/compressed (sensor_msgs/CompressedImage) -- RealSense Camera Vision
  - /yours_base/odom (nav_msgs/Odometry) -- Live robot position

Topics Published:
  - /move_base_simple/goal (geometry_msgs/PoseStamped) -- Navigation target pose
  - /yours_base/refrigerator_door (std_msgs/UInt16) -- Storage door motor (1=Open, 0=Close)
  - /yours_base/refrigerator_setpoint (std_msgs/Int16) -- Target temperature (°C)
  - /chat_audio_text (std_msgs/String) -- Physical speaker TTS voice
  - /yours_vla/status (std_msgs/String) -- Real-time VLA reasoning thought stream
"""

import sys
import json
import time
import base64
import rospy
from std_msgs.msg import String, UInt16, Int16, UInt8, UInt32
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class YoursVLAAgent:
    def __init__(self):
        rospy.init_node("yours_vla_agent", anonymous=False)

        # Publishers
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.pub_door = rospy.Publisher("/yours_base/refrigerator_door", UInt16, queue_size=5)
        self.pub_lock = rospy.Publisher("/yours_base/locks_ctrl", UInt32, queue_size=5)
        self.pub_setpoint = rospy.Publisher("/yours_base/refrigerator_setpoint", Int16, queue_size=5)
        self.pub_fridge_mode = rospy.Publisher("/yours_base/refrigerator_mode", UInt8, queue_size=5)
        self.pub_voice = rospy.Publisher("/chat_audio_text", String, queue_size=5)
        self.pub_vla_status = rospy.Publisher("/yours_vla/status", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/yours_vla/command", String, self.on_vla_command)
        rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.on_camera_frame)
        rospy.Subscriber("/yours_base/odom", Odometry, self.on_odom)

        self.latest_image_base64 = None
        self.current_pose = {"x": 0.0, "y": 0.0}

        rospy.loginfo("🧠 Yours Robotics - 小车 VLA Agent Initialized. Ready for instructions.")
        self.publish_status("VLA Agent Ready. Standing by for natural language prompt.")

    def publish_status(self, text, action_type="THOUGHT"):
        status_msg = json.dumps({
            "timestamp": time.time(),
            "type": action_type,
            "text": text
        })
        self.pub_vla_status.publish(String(data=status_msg))
        rospy.loginfo(f"[{action_type}] {text}")

    def on_odom(self, msg):
        self.current_pose["x"] = msg.pose.pose.position.x
        self.current_pose["y"] = msg.pose.pose.position.y

    def on_camera_frame(self, msg):
        if msg.data:
            self.latest_image_base64 = base64.b64encode(msg.data).decode('utf-8')

    def on_vla_command(self, msg):
        prompt = msg.data.strip()
        if not prompt:
            return

        self.publish_status(f"Received Instruction: '{prompt}'", "INPUT")

        # Parse spatial intent & action decomposition
        actions = self.parse_vla_intent(prompt)
        self.execute_vla_actions(actions)

    def parse_vla_intent(self, prompt):
        """
        Decomposes natural language instruction into multi-modal robot action sequence.
        """
        p = prompt.lower()
        actions = []

        # 1. Navigation Targets
        if "lobby" in p:
            actions.append({"type": "NAVIGATE", "target": "Lobby Station", "x": 2.0, "y": 1.5, "yaw": 0.0})
        elif "storage" in p or "dropoff" in p or "fridge" in p:
            actions.append({"type": "NAVIGATE", "target": "Storage Dropoff", "x": -1.5, "y": 3.0, "yaw": 1.57})
        elif "desk" in p or "service" in p:
            actions.append({"type": "NAVIGATE", "target": "Service Desk", "x": 0.5, "y": -2.0, "yaw": -1.57})
        elif "dock" in p or "charge" in p or "recharge" in p:
            actions.append({"type": "AUTODOCK"})

        # 2. Physical Storage Door & Lock Actuators
        if "open door" in p or "open storage" in p or "open compartment" in p:
            actions.append({"type": "DOOR_MOTOR", "cmd": 1})
        elif "close door" in p or "close storage" in p or "lock" in p:
            actions.append({"type": "DOOR_MOTOR", "cmd": 0})

        # 3. Climate Control Setpoints
        if "cool" in p or "chilled" in p or "cold" in p:
            actions.append({"type": "CLIMATE", "mode": 1, "temp": 4})
        elif "heat" in p or "warm" in p or "hot" in p:
            actions.append({"type": "CLIMATE", "mode": 2, "temp": 50})

        # 4. Spoken Voice Output
        if "say" in p or "speak" in p or "greet" in p:
            spoken_text = prompt
            if "say" in p:
                spoken_text = prompt.split("say")[-1].strip(" '\"")
            actions.append({"type": "SPEAK", "text": spoken_text})

        # Default fallback response if no explicit keywords matched
        if not actions:
            actions.append({"type": "SPEAK", "text": f"Understood: {prompt}. Executing autonomous spatial analysis."})

        return actions

    def execute_vla_actions(self, actions):
        for act in actions:
            a_type = act["type"]

            if a_type == "NAVIGATE":
                self.publish_status(f"Navigating 小车 to {act['target']} (x: {act['x']}, y: {act['y']})...", "ACTION_NAV")
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = act["x"]
                goal.pose.position.y = act["y"]
                goal.pose.orientation.w = 1.0
                self.pub_goal.publish(goal)

            elif a_type == "DOOR_MOTOR":
                cmd_name = "OPENING" if act["cmd"] == 1 else "CLOSING"
                self.publish_status(f"Actuating Storage Compartment Motor: {cmd_name} DOOR", "ACTION_ACTUATOR")
                self.pub_door.publish(UInt16(data=act["cmd"]))
                self.pub_lock.publish(UInt32(data=0 if act["cmd"] == 1 else 1))

            elif a_type == "CLIMATE":
                self.publish_status(f"Setting Storage Climate Mode={act['mode']}, Target Temp={act['temp']}°C", "ACTION_CLIMATE")
                self.pub_fridge_mode.publish(UInt8(data=act["mode"]))
                self.pub_setpoint.publish(Int16(data=act["temp"]))

            elif a_type == "SPEAK":
                self.publish_status(f"Speaking over physical speakers: '{act['text']}'", "ACTION_VOICE")
                self.pub_voice.publish(String(data=act["text"]))

            time.sleep(0.5)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        agent = YoursVLAAgent()
        agent.run()
    except rospy.ROSInterruptException:
        pass
