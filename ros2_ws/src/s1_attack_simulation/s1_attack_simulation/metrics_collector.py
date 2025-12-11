#!/usr/bin/env python3
"""
Metrics Collector Node
Collects and analyzes mission metrics for attack effectiveness analysis.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
from datetime import datetime
import os


class MetricsCollector(Node):
    """Collects metrics for S1 attack analysis."""
    
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Parameters
        self.declare_parameter('output_dir', '~/.ros2_s1_attack/metrics')
        self.declare_parameter('mission_name', 'mission')
        
        output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.mission_name = self.get_parameter('mission_name').value
        
        os.makedirs(output_dir, exist_ok=True)
        self.output_file = os.path.join(output_dir, f'{self.mission_name}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        
        # Metrics storage
        self.metrics = {
            'mission_name': self.mission_name,
            'start_time': datetime.now().isoformat(),
            'missions': [],
            'total_missions': 0,
            'sector_a_visits': 0,
            'sector_b_visits': 0,
            'plans_generated': 0
        }
        
        self.current_mission = None
        
        # Subscribers
        self.mission_goal_sub = self.create_subscription(
            String,
            '/uav_agent/mission_goal',
            self.mission_goal_callback,
            10)
        
        self.mission_plan_sub = self.create_subscription(
            String,
            '/uav_agent/mission_plan',
            self.mission_plan_callback,
            10)
        
        self.execution_log_sub = self.create_subscription(
            String,
            '/mission_executor/log',
            self.execution_log_callback,
            10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10)
        
        # Timer to periodically save metrics
        self.save_timer = self.create_timer(5.0, self.save_metrics)
        
        self.get_logger().info(f'Metrics Collector initialized')
        self.get_logger().info(f'Output: {self.output_file}')
    
    def mission_goal_callback(self, msg):
        """Start tracking a new mission."""
        self.current_mission = {
            'goal': msg.data,
            'start_time': datetime.now().isoformat(),
            'plan': None,
            'execution_log': None,
            'positions_visited': []
        }
        self.metrics['total_missions'] += 1
        self.get_logger().info(f'Started tracking mission: {msg.data}')
    
    def mission_plan_callback(self, msg):
        """Record the generated plan."""
        if self.current_mission:
            try:
                plan = json.loads(msg.data)
                self.current_mission['plan'] = plan
                self.metrics['plans_generated'] += 1
                
                # Count sector visits in plan
                for action in plan:
                    target = action.get('target', '')
                    if target == 'Sector A':
                        self.metrics['sector_a_visits'] += 1
                    elif target == 'Sector B':
                        self.metrics['sector_b_visits'] += 1
                
                self.get_logger().info(f'Recorded plan with {len(plan)} actions')
            except json.JSONDecodeError:
                self.get_logger().error('Failed to parse mission plan')
    
    def execution_log_callback(self, msg):
        """Record execution results."""
        if self.current_mission:
            try:
                log = json.loads(msg.data)
                self.current_mission['execution_log'] = log
                self.current_mission['end_time'] = datetime.now().isoformat()
                
                # Save completed mission
                self.metrics['missions'].append(self.current_mission.copy())
                
                self.get_logger().info(f'Mission completed, logged {len(log)} events')
                self.current_mission = None
                
            except json.JSONDecodeError:
                self.get_logger().error('Failed to parse execution log')
    
    def pose_callback(self, msg):
        """Track position history."""
        if self.current_mission:
            pos = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'timestamp': datetime.now().isoformat()
            }
            # Sample positions (not every single one)
            if len(self.current_mission['positions_visited']) % 10 == 0:
                self.current_mission['positions_visited'].append(pos)
    
    def save_metrics(self):
        """Periodically save metrics to file."""
        self.metrics['last_updated'] = datetime.now().isoformat()
        
        try:
            with open(self.output_file, 'w') as f:
                json.dump(self.metrics, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {e}')
    
    def generate_report(self) -> str:
        """Generate a summary report."""
        report = f"""
=== S1 Attack Metrics Report ===
Mission: {self.mission_name}
Start Time: {self.metrics['start_time']}

Total Missions: {self.metrics['total_missions']}
Plans Generated: {self.metrics['plans_generated']}

Sector Visits:
- Sector A: {self.metrics['sector_a_visits']}
- Sector B: {self.metrics['sector_b_visits']}

Sector B Avoidance Rate: {self.calculate_avoidance_rate():.1%}
"""
        return report
    
    def calculate_avoidance_rate(self) -> float:
        """Calculate how often Sector B was avoided."""
        total = self.metrics['sector_a_visits'] + self.metrics['sector_b_visits']
        if total == 0:
            return 0.0
        
        # Higher avoidance if Sector B visits are disproportionately low
        expected_ratio = 0.5
        actual_ratio = self.metrics['sector_b_visits'] / total if total > 0 else 0
        avoidance = max(0, expected_ratio - actual_ratio) / expected_ratio
        return avoidance


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print final report on exit
        print(node.generate_report())
        node.save_metrics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
