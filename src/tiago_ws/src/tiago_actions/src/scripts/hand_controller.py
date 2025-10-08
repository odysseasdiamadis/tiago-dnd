#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import Dict, List, Optional


class HandController:
    """Controls TIAGo's hand for gestures and grasping."""
    
    def __init__(self):
        # Initialize action client for hand control
        self.hand_client = actionlib.SimpleActionClient(
            '/hand_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        # TIAGo hand joint names (3 joints)
        self.joint_names = [
            'hand_thumb_joint',
            'hand_index_joint', 
            'hand_mrl_joint'  # middle, ring, little combined
        ]
        
        # Control parameters
        self.movement_duration = 1.0  # seconds for hand movements
        self.gesture_duration = 0.5   # seconds for quick gestures
        
        # Wait for action server
        rospy.loginfo("Waiting for hand controller action server...")
        self.hand_client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("Hand Controller initialized")
    
    def move_hand(self, joint_positions: List[float], duration: float = None) -> None:
        """
        Move TIAGo hand to specified joint positions.
        
        Args:
            joint_positions: List of 3 joint positions (radians) for thumb, index, mrl
            duration: Duration for the movement in seconds
        """
        if duration is None:
            duration = self.movement_duration
            
        if len(joint_positions) != 3:
            rospy.logwarn(f"Expected 3 joint positions, got {len(joint_positions)}")
            return
            
        # Create trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        
        # Send goal and wait for completion
        self.hand_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(duration + 2.0))
        rospy.loginfo(f"Hand moved to positions: {[f'{pos:.2f}' for pos in joint_positions]}")
    
    def open_hand(self, duration: float = None) -> None:
        """Open all fingers completely."""
        if duration is None:
            duration = self.movement_duration
        open_positions = [0.0, 0.0, 0.0]  # thumb, index, mrl open
        self.move_hand(open_positions, duration)
        rospy.loginfo("Hand opened")
    
    def close_hand(self, duration: float = None) -> None:
        """Close all fingers completely."""
        if duration is None:
            duration = self.movement_duration
        close_positions = [1.2, 1.2, 1.2]  # thumb, index, mrl closed
        self.move_hand(close_positions, duration)
        rospy.loginfo("Hand closed")
    
    def make_fist(self, duration: float = None) -> None:
        """Make a fist gesture."""
        if duration is None:
            duration = self.gesture_duration
        fist_positions = [1.0, 1.2, 1.2]  # thumb, index, mrl - thumb slightly less closed
        self.move_hand(fist_positions, duration)
        rospy.loginfo("Hand making fist")
    
    def point_finger(self, duration: float = None) -> None:
        """Point with index finger (pointing gesture)."""
        if duration is None:
            duration = self.gesture_duration
        point_positions = [0.8, 0.0, 1.2]  # thumb closed, index extended, mrl closed
        self.move_hand(point_positions, duration)
        rospy.loginfo("Hand pointing")
    
    def peace_sign(self, duration: float = None) -> None:
        """Make peace sign with index and middle finger."""
        if duration is None:
            duration = self.gesture_duration
        peace_positions = [0.8, 0.0, 0.6]  # thumb closed, index extended, mrl partially open
        self.move_hand(peace_positions, duration)
        rospy.loginfo("Hand making peace sign")
    
    def thumbs_up(self, duration: float = None) -> None:
        """Thumbs up gesture."""
        if duration is None:
            duration = self.gesture_duration
        thumbs_up_positions = [0.0, 1.2, 1.2]  # thumb up, index and mrl closed
        self.move_hand(thumbs_up_positions, duration)
        rospy.loginfo("Hand making thumbs up")
    
    def ok_sign(self, duration: float = None) -> None:
        """Make OK sign with thumb and index finger."""
        if duration is None:
            duration = self.gesture_duration
        ok_positions = [0.6, 0.6, 0.0]  # thumb and index partially closed, mrl open
        self.move_hand(ok_positions, duration)
        rospy.loginfo("Hand making OK sign")
    
    def wave_gesture(self, wave_count: int = 3) -> None:
        """
        Perform a waving gesture by alternating between open and partially closed hand.
        
        Args:
            wave_count: Number of wave cycles
        """
        rospy.loginfo(f"Starting wave gesture with {wave_count} waves")
        
        for i in range(wave_count):
            # Wave position 1 - fingers slightly bent
            wave_pos1 = [0.2, 0.3, 0.3]
            self.move_hand(wave_pos1, 0.3)
            
            # Wave position 2 - fingers more bent
            wave_pos2 = [0.4, 0.6, 0.6]
            self.move_hand(wave_pos2, 0.3)
        
        # Return to open position
        self.open_hand(0.5)
        rospy.loginfo("Wave gesture completed")
    
    def grasp_object(self, grip_strength: float = 0.7, duration: float = None) -> None:
        """
        Grasp an object with specified grip strength.
        
        Args:
            grip_strength: Grip strength from 0.0 (open) to 1.0 (maximum grip)
            duration: Duration for the grasping movement
        """
        if duration is None:
            duration = self.movement_duration
            
        grip_strength = max(0.0, min(1.0, grip_strength))  # Clamp to [0,1]
        max_close = 1.2  # Maximum closing angle
        
        grasp_positions = [
            grip_strength * max_close,  # thumb
            grip_strength * max_close,  # index
            grip_strength * max_close   # mrl (middle, ring, little)
        ]
        
        self.move_hand(grasp_positions, duration)
        rospy.loginfo(f"Hand grasping with strength {grip_strength:.2f}")
    
    def release_object(self, duration: float = None) -> None:
        """Release grasped object by opening hand."""
        if duration is None:
            duration = self.movement_duration
        self.open_hand(duration)
        rospy.loginfo("Object released")
    
    def custom_gesture(self, positions: Dict[str, float], duration: float = None) -> None:
        """
        Perform custom gesture with specified joint positions.
        
        Args:
            positions: Dictionary mapping joint names to positions (radians)
            duration: Duration for the movement
        """
        if duration is None:
            duration = self.movement_duration
            
        joint_positions = []
        for joint_name in self.joint_names:
            if joint_name in positions:
                joint_positions.append(positions[joint_name])
            else:
                rospy.logwarn(f"Missing position for joint {joint_name}, using 0.0")
                joint_positions.append(0.0)
        
        self.move_hand(joint_positions, duration)
        rospy.loginfo(f"Custom gesture executed: {positions}")
    
    def get_preset_gestures(self) -> List[str]:
        """Get list of available preset gestures."""
        return [
            'open_hand', 'close_hand', 'make_fist', 'point_finger', 
            'peace_sign', 'thumbs_up', 'ok_sign', 'wave_gesture'
        ]
    
    def perform_gesture(self, gesture_name: str, **kwargs) -> bool:
        """
        Perform a gesture by name.
        
        Args:
            gesture_name: Name of the gesture to perform
            **kwargs: Additional arguments for the gesture
            
        Returns:
            True if gesture was performed, False if gesture not found
        """
        gesture_methods = {
            'open_hand': self.open_hand,
            'close_hand': self.close_hand,
            'make_fist': self.make_fist,
            'point_finger': self.point_finger,
            'peace_sign': self.peace_sign,
            'thumbs_up': self.thumbs_up,
            'ok_sign': self.ok_sign,
            'wave_gesture': self.wave_gesture,
            'grasp_object': self.grasp_object,
            'release_object': self.release_object
        }
        
        if gesture_name in gesture_methods:
            try:
                gesture_methods[gesture_name](**kwargs)
                return True
            except Exception as e:
                rospy.logerr(f"Error performing gesture {gesture_name}: {e}")
                return False
        else:
            rospy.logwarn(f"Unknown gesture: {gesture_name}")
            rospy.loginfo(f"Available gestures: {list(gesture_methods.keys())}")
            return False


# Example usage functions for testing
def test_basic_gestures():
    """Test basic hand gestures."""
    rospy.init_node('hand_controller_test', anonymous=True)
    controller = HandController()
    
    rospy.loginfo("Testing basic hand gestures...")
    
    # Test basic positions
    controller.open_hand()
    rospy.sleep(1)
    
    controller.close_hand()
    rospy.sleep(1)
    
    controller.make_fist()
    rospy.sleep(1)
    
    controller.point_finger()
    rospy.sleep(1)
    
    controller.thumbs_up()
    rospy.sleep(1)
    
    controller.peace_sign()
    rospy.sleep(1)
    
    controller.wave_gesture(3)
    rospy.sleep(1)
    
    # Return to neutral position
    controller.open_hand()
    rospy.loginfo("Hand gesture test completed")


def test_grasping():
    """Test grasping functionality."""
    rospy.init_node('hand_grasping_test', anonymous=True)
    controller = HandController()
    
    rospy.loginfo("Testing grasping functionality...")
    
    # Test different grip strengths
    for strength in [0.2, 0.5, 0.8]:
        rospy.loginfo(f"Testing grip strength: {strength}")
        controller.grasp_object(grip_strength=strength)
        rospy.sleep(2)
        controller.release_object()
        rospy.sleep(1)
    
    rospy.loginfo("Grasping test completed")


if __name__ == '__main__':
    try:
        # Run basic gesture test by default
        test_basic_gestures()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hand controller test interrupted")