#!/usr/bin/env python3
"""
Mission Executor Node
Executes waypoint missions by interfacing with PX4 via MAVROS.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from nav_msgs.msg import Path
import json
import time
from enum import Enum


class MissionState(Enum):
    """Mission execution states."""
    IDLE = 0
    ARMING = 1
    TAKING_OFF = 2
    EXECUTING_WAYPOINTS = 3
    RETURNING = 4
    LANDING = 5
    COMPLETED = 6
    FAILED = 7


class MissionExecutor(Node):
    """Executes flight missions through PX4/MAVROS."""
    
    def __init__(self):
        super().__init__('mission_executor')
        
        # Parameters
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('sector_a_position', [10.0, 0.0, 5.0])
        self.declare_parameter('sector_b_position', [-10.0, 0.0, 5.0])
        self.declare_parameter('base_position', [0.0, 0.0, 0.0])
        
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.sector_positions = {
            'Sector A': self.get_parameter('sector_a_position').value,
            'Sector B': self.get_parameter('sector_b_position').value,
            'Base': self.get_parameter('base_position').value
        }
        
        # State
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.mission_state = MissionState.IDLE
        self.current_waypoint_index = 0
        self.current_mission = []
        self.mission_log = []
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile)
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile)
        
        self.mission_plan_sub = self.create_subscription(
            String,
            '/uav_agent/mission_plan',
            self.mission_plan_callback,
            10)
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)
        
        self.status_pub = self.create_publisher(
            String,
            '/mission_executor/status',
            10)
        
        self.execution_log_pub = self.create_publisher(
            String,
            '/mission_executor/log',
            10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Timer to send setpoints before arming
        self.setpoint_timer = self.create_timer(0.05, self.send_initial_setpoint)
        
        self.get_logger().info('Mission Executor initialized')
    
    def state_callback(self, msg):
        """Callback for MAVROS state updates."""
        self.current_state = msg
    
    def pose_callback(self, msg):
        """Callback for position updates."""
        self.current_pose = msg
    
    def mission_plan_callback(self, msg):
        """Receive mission plan from UAV agent."""
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f'Received mission plan with {len(plan)} actions')
            self.current_mission = plan
            self.mission_log = []
            self.current_waypoint_index = 0
            self.mission_state = MissionState.ARMING
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse mission plan: {e}')
    
    def send_initial_setpoint(self):
        """Send setpoints before arming (required by PX4)."""
        if self.mission_state == MissionState.IDLE or not self.current_state.armed:
            # Send current position to establish setpoint stream
            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg()
            setpoint.header.frame_id = 'map'
            setpoint.pose.position.x = 0.0
            setpoint.pose.position.y = 0.0
            setpoint.pose.position.z = self.takeoff_altitude
            self.setpoint_pub.publish(setpoint)
    
    def control_loop(self):
        """Main control loop for mission execution."""
        if self.mission_state == MissionState.ARMING:
            self.handle_arming()
        elif self.mission_state == MissionState.TAKING_OFF:
            self.handle_takeoff()
        elif self.mission_state == MissionState.EXECUTING_WAYPOINTS:
            self.handle_waypoint_execution()
        elif self.mission_state == MissionState.RETURNING:
            self.handle_return()
        elif self.mission_state == MissionState.LANDING:
            self.handle_landing()
        elif self.mission_state == MissionState.COMPLETED:
            self.handle_completion()
    
    def handle_arming(self):
        """Arm the vehicle and set OFFBOARD mode."""
        if not self.current_state.connected:
            self.get_logger().warn('Waiting for FCU connection...', throttle_duration_sec=5.0)
            return
        
        if not self.current_state.armed:
            self.get_logger().info('Attempting to arm...')
            if self.call_arming_service(True):
                self.get_logger().info('Armed successfully')
        
        if self.current_state.mode != "OFFBOARD":
            self.get_logger().info('Setting OFFBOARD mode...')
            if self.call_set_mode_service("OFFBOARD"):
                self.get_logger().info('OFFBOARD mode set')
        
        if self.current_state.armed and self.current_state.mode == "OFFBOARD":
            self.mission_state = MissionState.TAKING_OFF
            self.get_logger().info('Transitioning to TAKEOFF')
    
    def handle_takeoff(self):
        """Handle takeoff to specified altitude."""
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = self.takeoff_altitude
        
        self.setpoint_pub.publish(target_pose)
        
        # Check if reached altitude
        if abs(self.current_pose.pose.position.z - self.takeoff_altitude) < self.waypoint_tolerance:
            self.get_logger().info('Takeoff complete, starting mission')
            self.mission_state = MissionState.EXECUTING_WAYPOINTS
            self.add_to_log('TAKEOFF: Reached altitude')
    
    def handle_waypoint_execution(self):
        """Execute waypoints from mission plan."""
        if self.current_waypoint_index >= len(self.current_mission):
            self.get_logger().info('All waypoints completed')
            self.mission_state = MissionState.RETURNING
            return
        
        action = self.current_mission[self.current_waypoint_index]
        action_type = action.get('action', '')
        target = action.get('target', '')
        
        if action_type == 'FLY_TO' or action_type == 'SCAN_SECTOR':
            if target in self.sector_positions:
                target_pos = self.sector_positions[target]
                
                # Create setpoint
                setpoint = PoseStamped()
                setpoint.header.stamp = self.get_clock().now().to_msg()
                setpoint.header.frame_id = 'map'
                setpoint.pose.position.x = target_pos[0]
                setpoint.pose.position.y = target_pos[1]
                setpoint.pose.position.z = target_pos[2]
                
                self.setpoint_pub.publish(setpoint)
                
                # Check if reached waypoint
                current_pos = self.current_pose.pose.position
                distance = ((current_pos.x - target_pos[0])**2 +
                           (current_pos.y - target_pos[1])**2 +
                           (current_pos.z - target_pos[2])**2)**0.5
                
                if distance < self.waypoint_tolerance:
                    log_msg = f'{action_type}: {target} (pos: {target_pos})'
                    self.get_logger().info(log_msg)
                    self.add_to_log(log_msg)
                    self.current_waypoint_index += 1
            else:
                self.get_logger().warn(f'Unknown target: {target}')
                self.current_waypoint_index += 1
        
        elif action_type == 'RETURN_TO_BASE':
            self.mission_state = MissionState.RETURNING
        
        else:
            self.get_logger().warn(f'Unknown action: {action_type}')
            self.current_waypoint_index += 1
    
    def handle_return(self):
        """Return to base position."""
        base_pos = self.sector_positions['Base']
        
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        setpoint.pose.position.x = base_pos[0]
        setpoint.pose.position.y = base_pos[1]
        setpoint.pose.position.z = self.takeoff_altitude
        
        self.setpoint_pub.publish(setpoint)
        
        # Check if at base
        current_pos = self.current_pose.pose.position
        distance = ((current_pos.x - base_pos[0])**2 +
                   (current_pos.y - base_pos[1])**2)**0.5
        
        if distance < self.waypoint_tolerance:
            self.get_logger().info('Reached base, landing')
            self.add_to_log('RETURN: Reached base')
            self.mission_state = MissionState.LANDING
    
    def handle_landing(self):
        """Land the vehicle."""
        # Descend gradually
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        setpoint.pose.position.x = 0.0
        setpoint.pose.position.y = 0.0
        setpoint.pose.position.z = 0.0
        
        self.setpoint_pub.publish(setpoint)
        
        # Check if landed
        if self.current_pose.pose.position.z < 0.3:
            self.get_logger().info('Landing complete, disarming')
            self.call_arming_service(False)
            self.add_to_log('LANDING: Complete')
            self.mission_state = MissionState.COMPLETED
    
    def handle_completion(self):
        """Handle mission completion."""
        # Publish execution log
        log_msg = String()
        log_msg.data = json.dumps(self.mission_log)
        self.execution_log_pub.publish(log_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = "MISSION_COMPLETE"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('Mission complete!')
        self.mission_state = MissionState.IDLE
    
    def add_to_log(self, message: str):
        """Add message to mission log."""
        self.mission_log.append(message)
        
    def call_arming_service(self, arm: bool) -> bool:
        """Call the arming service."""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming service not available')
            return False
        
        request = CommandBool.Request()
        request.value = arm
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def call_set_mode_service(self, mode: str) -> bool:
        """Call the set mode service."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Set mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            return future.result().mode_sent
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
