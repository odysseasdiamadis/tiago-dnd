#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from typing import Optional, Dict, Tuple
import moveit_commander


class ArmController:
    """Controls TIAGo's arm using MoveIt for pose-based control."""
    
    def __init__(self):
        """Initialize the arm controller with MoveIt."""
        # Initialize moveit commander
        moveit_commander.roscpp_initialize([])
        
        # Setup MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm_torso")
        
        # Configure planning parameters
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.01)  # 1cm
        self.arm_group.set_goal_orientation_tolerance(0.1)  # ~6 degrees
        
        # Reference frame for planning
        self.reference_frame = "base_footprint"
        self.arm_group.set_pose_reference_frame(self.reference_frame)
        
        # Get end effector info
        self.end_effector_link = self.arm_group.get_end_effector_link()
        
        # Elbow constraint settings
        self.use_elbow_constraint = False
        self.elbow_down_constraint = None
        
        # Shoulder constraint settings (for natural pointing)
        self.use_shoulder_constraint = False
        self.shoulder_down_constraint = None
        
        rospy.sleep(0.5)
        rospy.loginfo("Arm Controller initialized with MoveIt")
        rospy.loginfo(f"End effector: {self.end_effector_link}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        self.set_natural_pointing_constraints(True)

    
    def set_elbow_down_constraint(self, enable: bool = True, max_elbow_angle: float = 0.0):
        self.use_elbow_constraint = enable
        
        if enable:
            # Create joint constraint for elbow (typically arm_2_joint for TIAGo)
            elbow_constraint = JointConstraint()
            elbow_constraint.joint_name = "arm_2_joint"  # TIAGo's elbow joint
            elbow_constraint.position = max_elbow_angle
            elbow_constraint.tolerance_above = 0.5  # Allow some flexibility above
            elbow_constraint.tolerance_below = 3.14  # Allow full range below
            elbow_constraint.weight = 1.0
            
            # Create constraints message
            self.elbow_down_constraint = Constraints()
            self.elbow_down_constraint.name = "elbow_down"
            self.elbow_down_constraint.joint_constraints = [elbow_constraint]
            
            rospy.loginfo(f"Elbow down constraint enabled (max angle: {max_elbow_angle:.3f} rad)")
        else:
            self.elbow_down_constraint = None
            rospy.loginfo("Elbow constraint disabled")
    
    def set_shoulder_down_constraint(self, enable: bool = True, max_shoulder_angle: float = 0.5):
        """
        Enable/disable constraint to keep shoulder pointing downward for natural pointing.
        
        Args:
            enable: Whether to enable the shoulder constraint
            max_shoulder_angle: Maximum angle for shoulder joint (radians).
                              Positive values allow shoulder to go up, 0.0 = level, negative = down
        """
        self.use_shoulder_constraint = enable
        
        if enable:
            # Create joint constraint for shoulder (arm_1_joint for TIAGo)
            shoulder_constraint = JointConstraint()
            shoulder_constraint.joint_name = "arm_1_joint"  # TIAGo's shoulder joint
            shoulder_constraint.position = max_shoulder_angle
            shoulder_constraint.tolerance_above = 0.3  # Allow some flexibility above
            shoulder_constraint.tolerance_below = 1.57  # Allow full range below (down)
            shoulder_constraint.weight = 0.8  # Slightly lower weight than elbow
            
            # Create constraints message
            self.shoulder_down_constraint = Constraints()
            self.shoulder_down_constraint.name = "shoulder_down"
            self.shoulder_down_constraint.joint_constraints = [shoulder_constraint]
            
            rospy.loginfo(f"Shoulder down constraint enabled (max angle: {max_shoulder_angle:.3f} rad)")
        else:
            self.shoulder_down_constraint = None
            rospy.loginfo("Shoulder constraint disabled")
    
    def set_natural_pointing_constraints(self, enable: bool = True):
        if enable:
            self.set_shoulder_down_constraint(True, max_shoulder_angle=0.3)  # Shoulder slightly down
            self.set_elbow_down_constraint(True, max_elbow_angle=-0.1)       # Elbow slightly bent down
            rospy.loginfo("Natural pointing constraints enabled")
        else:
            self.set_shoulder_down_constraint(False)
            self.set_elbow_down_constraint(False)
            rospy.loginfo("Natural pointing constraints disabled")
    
    def clear_constraints(self):
        """Clear all path constraints."""
        self.arm_group.clear_path_constraints()
        self.use_elbow_constraint = False
        self.use_shoulder_constraint = False
        rospy.loginfo("All path constraints cleared")
    
    def move_to_pose(self, position: Tuple[float, float, float], 
                     orientation: Tuple[float, float, float, float] = None,
                     rpy: Tuple[float, float, float] = None,
                     keep_elbow_down: bool = None) -> bool:
        """
        Move wrist to specified pose (position + orientation).
        
        Args:
            position: (x, y, z) position in meters relative to base_footprint
            orientation: (x, y, z, w) quaternion OR None if using rpy
            rpy: (roll, pitch, yaw) in radians OR None if using quaternion
            keep_elbow_down: Force elbow down for this movement (overrides global setting)
            
        Returns:
            True if successful, False otherwise
        """
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        target_pose.pose.position = Point(*position)
        
        # Set orientation
        if orientation is not None:
            target_pose.pose.orientation = Quaternion(*orientation)
        elif rpy is not None:
            quat = quaternion_from_euler(*rpy)
            target_pose.pose.orientation = Quaternion(*quat)
        else:
            # Default orientation (pointing forward)
            target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        # Apply constraints if needed
        use_elbow = keep_elbow_down if keep_elbow_down is not None else self.use_elbow_constraint
        
        # Combine constraints
        combined_constraints = Constraints()
        combined_constraints.name = "arm_constraints"
        has_constraints = False
        
        if use_elbow and self.elbow_down_constraint is not None:
            combined_constraints.joint_constraints.extend(self.elbow_down_constraint.joint_constraints)
            has_constraints = True
            rospy.loginfo("Applied elbow down constraint")
            
        if self.use_shoulder_constraint and self.shoulder_down_constraint is not None:
            combined_constraints.joint_constraints.extend(self.shoulder_down_constraint.joint_constraints)
            has_constraints = True
            rospy.loginfo("Applied shoulder down constraint")
        
        if has_constraints:
            self.arm_group.set_path_constraints(combined_constraints)
        else:
            self.arm_group.clear_path_constraints()
        
        # Plan and execute with MoveIt
        self.arm_group.set_pose_target(target_pose)
        
        rospy.loginfo(f"Planning movement to: pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
        
        plan = self.arm_group.plan()
        if len(plan[1].joint_trajectory.points) > 0:
            success = self.arm_group.execute(plan[1], wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            # Clear constraints after movement
            if has_constraints:
                self.arm_group.clear_path_constraints()
            
            if success:
                rospy.loginfo("Movement completed successfully")
                return True
            else:
                rospy.logwarn("Movement execution failed")
                return False
        else:
            rospy.logwarn("Failed to find valid plan")
            # Clear constraints if planning failed
            if has_constraints:
                self.arm_group.clear_path_constraints()
            return False
    
    def point_at(self, target_position: Tuple[float, float, float], 
                 wrist_distance: float = 0.3) -> bool:
        """
        Point the arm toward a target position.
        
        Args:
            target_position: (x, y, z) position to point at
            wrist_distance: Distance from target to place wrist (meters)
            
        Returns:
            True if successful, False otherwise
        """
        x_target, y_target, z_target = target_position
        
        # Calculate pointing direction
        pointing_vector = np.array([x_target, y_target, z_target])
        pointing_vector = pointing_vector / np.linalg.norm(pointing_vector)
        
        # Position wrist at specified distance from target
        wrist_x = x_target - wrist_distance * pointing_vector[0]
        wrist_y = y_target - wrist_distance * pointing_vector[1]
        wrist_z = z_target - wrist_distance * pointing_vector[2]
        
        # Calculate orientation to point toward target
        yaw = np.arctan2(pointing_vector[1], pointing_vector[0])
        pitch = -np.arcsin(pointing_vector[2])
        roll = 0.0  # Keep roll neutral
        
        rospy.loginfo(f"Pointing at ({x_target:.3f}, {y_target:.3f}, {z_target:.3f})")
        return self.move_to_pose((wrist_x, wrist_y, wrist_z), rpy=(roll, pitch, yaw))
    
    def point_at_player(self, player, arm_distance: float = 0.6, arm_height: float = 1.2, 
                       keep_elbow_down: bool = None) -> bool:
        """
        Point the arm towards a player based on their yaw angle.
        
        Args:
            player: Player object with yaw field
            arm_distance: Distance from robot base to place the arm (meters)
            arm_height: Height at which to position the arm (meters)
            keep_elbow_down: Force elbow down for this movement
            
        Returns:
            True if successful, False otherwise
        """
        # Create rotation matrix for yaw rotation around z-axis
        yaw = player.yaw
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # Rotation matrix for yaw around z-axis
        rotation_matrix = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])
        
        # Base direction vector [1, 0, 0] (pointing forward)
        base_vector = np.array([1, 0, -1.5])
        
        # Apply rotation to get the pointing direction
        pointing_direction = rotation_matrix.dot(base_vector)
        
        # Calculate arm position (pointing in the direction of the player)
        arm_x = arm_distance * pointing_direction[0]
        arm_y = arm_distance * pointing_direction[1]
        arm_z = arm_height
        
        # Set arm orientation to match the yaw (pointing towards player)
        # Roll and pitch are kept neutral, only yaw follows the player direction
        roll = 0.0
        pitch = 0.0  # Keep arm level
        arm_yaw = yaw  # Point in the same direction as the player
        
        rospy.loginfo(f"Pointing arm towards player at yaw {yaw:.3f} rad ({np.degrees(yaw):.1f} deg)")
        rospy.loginfo(f"Arm position: ({arm_x:.3f}, {arm_y:.3f}, {arm_z:.3f})")
        
        return self.move_to_pose((arm_x, arm_y, arm_z), rpy=(roll, pitch, arm_yaw), 
                               keep_elbow_down=keep_elbow_down)
    
    def move_relative(self, delta_position: Tuple[float, float, float] = (0, 0, 0),
                     delta_rpy: Tuple[float, float, float] = (0, 0, 0)) -> bool:
        """
        Move relative to current position.
        
        Args:
            delta_position: (dx, dy, dz) relative movement in meters
            delta_rpy: (d_roll, d_pitch, d_yaw) relative rotation in radians
            
        Returns:
            True if successful, False otherwise
        """
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
        
        # Calculate new position
        new_pos = (
            current_pose.position.x + delta_position[0],
            current_pose.position.y + delta_position[1],
            current_pose.position.z + delta_position[2]
        )
        
        # Calculate new orientation
        current_quat = [
            current_pose.orientation.x, current_pose.orientation.y,
            current_pose.orientation.z, current_pose.orientation.w
        ]
        current_rpy = euler_from_quaternion(current_quat)
        new_rpy = (
            current_rpy[0] + delta_rpy[0],
            current_rpy[1] + delta_rpy[1],
            current_rpy[2] + delta_rpy[2]
        )
        
        return self.move_to_pose(new_pos, rpy=new_rpy)
    
    def get_current_pose(self) -> Optional[Pose]:
        """Get current wrist pose."""
        try:
            current_pose = self.arm_group.get_current_pose(self.end_effector_link)
            return current_pose.pose
        except Exception as e:
            rospy.logwarn(f"Failed to get current pose: {e}")
            return None
    
    def get_predefined_poses(self) -> Dict[str, Dict]:
        """Get predefined useful poses."""
        return {
            'home': {
                'position': (0.3, 0.0, 1.0),
                'rpy': (0.0, 0.0, 0.0),
                'description': 'Neutral home position'
            },
            'point_forward': {
                'position': (0.6, 0.0, 1.2),
                'rpy': (0.0, -0.3, 0.0),
                'description': 'Point forward and slightly down'
            },
            'point_left': {
                'position': (0.3, 0.4, 1.2),
                'rpy': (0.0, 0.0, 1.57),
                'description': 'Point to the left'
            },
            'point_right': {
                'position': (0.3, -0.4, 1.2),
                'rpy': (0.0, 0.0, -1.57),
                'description': 'Point to the right'
            },
            'point_up': {
                'position': (0.3, 0.0, 1.6),
                'rpy': (0.0, -1.57, 0.0),
                'description': 'Point upward'
            },
            'rest': {
                'position': (0.2, -0.3, 0.9),
                'rpy': (0.0, 1.0, -1.0),
                'description': 'Resting position by the side'
            }
        }
    
    def move_to_preset(self, preset_name: str) -> bool:
        """
        Move to a predefined pose.
        
        Args:
            preset_name: Name of the preset pose
            
        Returns:
            True if successful, False otherwise
        """
        presets = self.get_predefined_poses()
        
        if preset_name not in presets:
            rospy.logwarn(f"Unknown preset: {preset_name}")
            rospy.loginfo(f"Available presets: {list(presets.keys())}")
            return False
        
        preset = presets[preset_name]
        rospy.loginfo(f"Moving to preset '{preset_name}': {preset['description']}")
        
        return self.move_to_pose(preset['position'], rpy=preset['rpy'])
    
    def stop(self):
        """Stop current movement."""
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.loginfo("Arm movement stopped")


# Example usage and test functions
def test_basic_movements():
    """Test basic arm movements."""
    rospy.init_node('arm_controller_test', anonymous=True)
    controller = ArmController()
    
    rospy.loginfo("Testing basic movements...")
    
    # Test predefined poses
    poses = ['home', 'point_forward', 'point_left', 'point_right', 'point_up', 'rest']
    
    for pose in poses:
        rospy.loginfo(f"Moving to {pose}")
        if controller.move_to_preset(pose):
            rospy.sleep(2)
        else:
            rospy.logwarn(f"Failed to move to {pose}")
    
    rospy.loginfo("Basic movement test completed")


def test_pointing():
    """Test pointing at specific positions."""
    rospy.init_node('arm_pointing_test', anonymous=True)
    controller = ArmController()
    
    rospy.loginfo("Testing pointing...")
    
    # Points to aim at
    targets = [
        (0.8, 0.0, 1.2),   # Front
        (0.5, 0.5, 1.0),   # Front-left
        (0.5, -0.5, 1.0),  # Front-right  
        (0.3, 0.0, 1.8),   # Up
        (0.3, 0.0, 0.6)    # Down
    ]
    
    for target in targets:
        rospy.loginfo(f"Pointing at {target}")
        if controller.point_at(target):
            rospy.sleep(2)
        else:
            rospy.logwarn(f"Failed to point at {target}")
    
    # Return home
    controller.move_to_preset('home')
    rospy.loginfo("Pointing test completed")


def test_player_pointing():
    """Test pointing at players."""
    rospy.init_node('arm_player_test', anonymous=True)
    controller = ArmController()
    
    # Import Player class for testing
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from player_model import Player
    
    rospy.loginfo("Testing player pointing...")
    
    # Create mock players at different yaw angles
    test_players = [
        Player([], 0.0, (0, 0, 0, 0), 0),        # Front (0 degrees)
        Player([], 1.57, (0, 0, 0, 0), 1),       # Left (90 degrees)
        Player([], -1.57, (0, 0, 0, 0), 2),      # Right (-90 degrees)
        Player([], 3.14, (0, 0, 0, 0), 3),       # Back (180 degrees)
        Player([], 0.785, (0, 0, 0, 0), 4),      # Front-left (45 degrees)
    ]
    
    for player in test_players:
        rospy.loginfo(f"Pointing at player {player.player_id} (yaw: {player.yaw:.3f} rad, {np.degrees(player.yaw):.1f} deg)")
        if controller.point_at_player(player):
            rospy.sleep(3)
        else:
            rospy.logwarn(f"Failed to point at player {player.player_id}")
    
    # Return home
    controller.move_to_preset('home')
    rospy.loginfo("Player pointing test completed")


def test_elbow_constraint():
    """Test elbow down constraint."""
    rospy.init_node('arm_elbow_test', anonymous=True)
    controller = ArmController()
    
    # Import Player class for testing
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from player_model import Player
    
    rospy.loginfo("Testing elbow constraint...")
    
    # Test without elbow constraint
    rospy.loginfo("=== Testing WITHOUT elbow constraint ===")
    controller.set_elbow_down_constraint(False)
    test_player = Player([], 1.57, (0, 0, 0, 0), 1)  # Left (90 degrees)
    controller.point_at_player(test_player)
    rospy.sleep(3)
    
    # Test with elbow constraint
    rospy.loginfo("=== Testing WITH elbow constraint ===")
    controller.set_elbow_down_constraint(True, max_elbow_angle=-0.2)  # Slight bend downward
    controller.point_at_player(test_player)
    rospy.sleep(3)
    
    # Test different positions with constraint
    rospy.loginfo("=== Testing different positions with elbow down ===")
    test_positions = [
        (0.5, 0.3, 1.1, "front-left low"),
        (0.4, -0.4, 1.4, "front-right high"),
        (0.6, 0.0, 0.9, "front low")
    ]
    
    for x, y, z, desc in test_positions:
        rospy.loginfo(f"Moving to {desc}: ({x}, {y}, {z})")
        controller.move_to_pose((x, y, z), rpy=(0, 0, 0), keep_elbow_down=True)
        rospy.sleep(2)
    
    # Clean up
    controller.clear_constraints()
    controller.move_to_preset('home')
    rospy.loginfo("Elbow constraint test completed")


def test_natural_pointing():
    """Test natural human-like pointing constraints."""
    rospy.init_node('arm_natural_pointing_test', anonymous=True)
    controller = ArmController()
    
    # Import Player class for testing
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from player_model import Player
    
    rospy.loginfo("Testing natural pointing constraints...")
    
    # Test regular pointing (no constraints)
    rospy.loginfo("=== Testing REGULAR pointing (no constraints) ===")
    controller.clear_constraints()
    test_player = Player([], 1.57, (0, 0, 0, 0), 1)  # Left (90 degrees)
    controller.point_at_player(test_player)
    rospy.sleep(4)
    
    # Test natural pointing constraints
    rospy.loginfo("=== Testing NATURAL pointing constraints ===")
    controller.set_natural_pointing_constraints(True)
    controller.point_at_player(test_player)
    rospy.sleep(4)
    
    # Test different positions with natural constraints
    rospy.loginfo("=== Testing different players with natural pointing ===")
    test_players = [
        Player([], 0.0, (0, 0, 0, 0), 0),        # Front (0 degrees)
        Player([], -1.57, (0, 0, 0, 0), 2),      # Right (-90 degrees)
        Player([], 0.785, (0, 0, 0, 0), 4),      # Front-left (45 degrees)
        Player([], -0.785, (0, 0, 0, 0), 5),     # Front-right (-45 degrees)
    ]
    
    for player in test_players:
        rospy.loginfo(f"Natural pointing at player {player.player_id} (yaw: {player.yaw:.3f} rad, {np.degrees(player.yaw):.1f} deg)")
        controller.point_at_player(player)
        rospy.sleep(3)
    
    # Clean up
    controller.clear_constraints()
    controller.move_to_preset('home')
    rospy.loginfo("Natural pointing test completed")


def demo_custom_poses():
    """Demo custom pose control."""
    rospy.init_node('arm_custom_demo', anonymous=True)
    controller = ArmController()
    
    rospy.loginfo("Demonstrating custom pose control...")
    
    # Move to specific position and orientation
    rospy.loginfo("Moving to custom position...")
    controller.move_to_pose(
        position=(0.4, 0.2, 1.3),
        rpy=(0.0, -0.5, 0.7)  # Roll, pitch, yaw in radians
    )
    rospy.sleep(2)
    
    # Move relatively
    rospy.loginfo("Moving relatively...")
    controller.move_relative(
        delta_position=(0.1, -0.1, 0.0),
        delta_rpy=(0.0, 0.0, -0.3)
    )
    rospy.sleep(2)
    
    # Return home
    controller.move_to_preset('home')
    rospy.loginfo("Custom pose demo completed")


if __name__ == '__main__':
    try:
        test_natural_pointing()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")