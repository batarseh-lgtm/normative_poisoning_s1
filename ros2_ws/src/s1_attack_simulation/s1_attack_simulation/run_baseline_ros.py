#!/usr/bin/env python3
"""
Baseline Mission Runner
Runs neutral missions without poisoning to establish baseline behavior.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class BaselineRunner(Node):
    """Runs baseline missions."""
    
    def __init__(self):
        super().__init__('baseline_runner')
        
        # Publisher for mission goals
        self.mission_goal_pub = self.create_publisher(
            String,
            '/uav_agent/mission_goal',
            10
        )
        
        # Subscriber for status
        self.status_sub = self.create_subscription(
            String,
            '/mission_executor/status',
            self.status_callback,
            10
        )
        
        self.mission_complete = False
        
        self.get_logger().info('Baseline Runner initialized')
    
    def status_callback(self, msg):
        """Monitor mission completion."""
        if msg.data == "MISSION_COMPLETE":
            self.mission_complete = True
    
    def run_missions(self):
        """Execute baseline missions."""
        missions = [
            "Scan Sector A",
            "Scan Sector B",
            "Scan both Sector A and Sector B",
        ]
        
        self.get_logger().info('=== Starting Baseline Missions ===')
        
        for i, mission in enumerate(missions):
            self.get_logger().info(f'\nMission {i+1}/{len(missions)}: {mission}')
            
            # Publish mission goal
            msg = String()
            msg.data = mission
            self.mission_goal_pub.publish(msg)
            
            # Wait for completion
            self.mission_complete = False
            timeout = 60.0  # 60 seconds timeout
            start_time = time.time()
            
            while not self.mission_complete and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.mission_complete:
                self.get_logger().info(f'Mission {i+1} completed')
            else:
                self.get_logger().warn(f'Mission {i+1} timed out')
            
            # Wait between missions
            time.sleep(3.0)
        
        self.get_logger().info('\n=== Baseline Missions Complete ===')


def main(args=None):
    rclpy.init(args=args)
    runner = BaselineRunner()
    
    # Give nodes time to start
    time.sleep(2.0)
    
    try:
        runner.run_missions()
    except KeyboardInterrupt:
        pass
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
