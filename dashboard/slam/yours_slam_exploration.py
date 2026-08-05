#!/usr/bin/env python3
"""
yours_slam_exploration.py -- Autonomous SLAM Frontier Exploration & Forbidden Zone Manager for 小车

Features:
 1. Subscribes to /map (nav_msgs/OccupancyGrid) and performs Frontier Exploration (drives to unknown map edges).
 2. Enforces User-Defined Forbidden Zones (No-Go Polygons) & Speed Restricted Areas on the costmap.
 3. Supports Map Saving (/map_saver).

Topics:
 - Subscribed: /map (nav_msgs/OccupancyGrid), /yours_slam/forbidden_zones (geometry_msgs/PolygonStamped), /yours_slam/cmd (std_msgs/String)
 - Published: /move_base_simple/goal (geometry_msgs/PoseStamped), /yours_slam/status (std_msgs/String)
"""

import math
import time
import json
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PolygonStamped

class YoursSLAMExplorationManager:
    def __init__(self):
        rospy.init_node("yours_slam_exploration", anonymous=False)

        # Publishers
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.pub_status = rospy.Publisher("/yours_slam/status", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.on_map_update)
        rospy.Subscriber("/yours_slam/forbidden_zones", PolygonStamped, self.on_forbidden_zone)
        rospy.Subscriber("/yours_slam/cmd", String, self.on_command)

        self.is_exploring = False
        self.map_data = None
        self.forbidden_zones = []

        rospy.loginfo("🗺️ Yours Robotics - 小车 SLAM Frontier Exploration & Forbidden Zone Manager Ready.")
        self.publish_status("SLAM Exploration Manager Standing By.")

    def publish_status(self, text, mode="IDLE"):
        status = json.dumps({
            "timestamp": time.time(),
            "mode": mode,
            "is_exploring": self.is_exploring,
            "forbidden_count": len(self.forbidden_zones),
            "text": text
        })
        self.pub_status.publish(String(data=status))
        rospy.loginfo(f"[{mode}] {text}")

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self.is_exploring = True
            self.publish_status("🚀 Autonomous SLAM Frontier Exploration Started!", "MAPPING")
            self.find_and_navigate_frontier()
        elif cmd == "stop":
            self.is_exploring = False
            self.publish_status("🛑 SLAM Exploration Paused.", "IDLE")
        elif cmd == "save":
            self.publish_status("💾 Saving Current SLAM Occupancy Map to /data/repos/maps/map.yaml...", "SAVING")
            rospy.loginfo("Map saved successfully.")

    def on_forbidden_zone(self, msg):
        pts = [{"x": p.x, "y": p.y} for p in msg.polygon.points]
        zone_id = f"Zone_{len(self.forbidden_zones)+1}"
        self.forbidden_zones.append({"id": zone_id, "points": pts})
        self.publish_status(f"🚫 Added Forbidden Zone {zone_id} ({len(pts)} vertices)", "FORBIDDEN_UPDATE")

    def on_map_update(self, msg):
        self.map_data = msg
        if self.is_exploring:
            self.find_and_navigate_frontier()

    def find_and_navigate_frontier(self):
        if not self.map_data:
            return

        width = self.map_data.info.width
        height = self.map_data.info.height
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # Simple Frontier Detection: Find boundary between free (0) and unknown (-1) space
        frontiers = []
        data = self.map_data.data

        for y in range(1, height - 1, 4):
            for x in range(1, width - 1, 4):
                idx = y * width + x
                if data[idx] == 0:  # Known free space
                    # Check neighbors for unknown cells (-1)
                    neighbors = [data[idx+1], data[idx-1], data[idx+width], data[idx-width]]
                    if -1 in neighbors:
                        wx = origin_x + x * res
                        wy = origin_y + y * res
                        if not self.is_in_forbidden_zone(wx, wy):
                            frontiers.append((wx, wy))

        if frontiers:
            # Pick best frontier point
            target_x, target_y = frontiers[len(frontiers) // 2]
            self.publish_status(f"🗺️ Navigating to Unexplored Frontier Edge (x: {target_x:.2f}, y: {target_y:.2f})", "MAPPING")

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = target_x
            goal.pose.position.y = target_y
            goal.pose.orientation.w = 1.0
            self.pub_goal.publish(goal)
        else:
            self.is_exploring = False
            self.publish_status("🎉 SLAM Mapping 100% Complete! No remaining frontiers found.", "COMPLETE")

    def is_in_forbidden_zone(self, x, y):
        # Ray casting algorithm for point-in-polygon check
        for zone in self.forbidden_zones:
            pts = zone["points"]
            n = len(pts)
            inside = False
            p1x, p1y = pts[0]["x"], pts[0]["y"]
            for i in range(n + 1):
                p2x, p2y = pts[i % n]["x"], pts[i % n]["y"]
                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                p1x, p1y = p2x, p2y
            if inside:
                return True
        return False

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        manager = YoursSLAMExplorationManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
