"""
TESTS ALL ROBOT GESTURES IN THE FILE
"""

import rospy
import numpy as np
import time
import csv
from datetime import datetime
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass, asdict
import sys
import os

from tiago_ws.src.tiago_actions.src.scripts.arm_controller import ArmController

from tiago_ws.src.tiago_actions.src.scripts.player_model import Player

# NOTE: we put this file here to have all benchmarks (except the main scene one)
#       to run this, you need to activate tiago_ws prerequisites before, see README!!


PAUSE_BETWEEN_TESTS = 0.5   # seconds

@dataclass
class BenchmarkResult:
    """Data class to store benchmark results for a single test."""
    timestamp: str
    run_number: int
    test_name: str
    test_category: str
    success: bool
    # planning_time: float  # to plan the motion TODO
    # execution_time: float  # to execute the motion TODO
    total_time: float  # Total time (planning + execution)
    target_position: str  # Target pos as str
    target_orientation: str  # Target orientation as str
    constraint_type: str  # Type of constraint used (None if there isn't one)
    error_message: str  # error message if failed


class ArmControllerBenchmark:
    """Benchmark for ArmController."""
    
    def __init__(self, output_file: str):
        """
        Initialize benchmark suite.
        output_file: Path to output TSV file. If None, generates timestamped filename.
        """
        rospy.init_node('arm_controller_benchmark', anonymous=True)
        self.controller = ArmController()
        
        self.results: List[BenchmarkResult] = []
        
        self.output_file = output_file
        rospy.loginfo(f"Benchmark initialized. Results at: {output_file}")
        

    def create_result(self, run_number: int, test_name: str, test_category: str,
                       success: bool, total_time: float,
                       target_pos: Tuple, target_ori: Any = None,
                       constraint_type: str = "none", error_msg: str = "") -> BenchmarkResult:
        """Create a BenchmarkResult obj"""

        return BenchmarkResult(
            timestamp=datetime.now().isoformat(),
            run_number=run_number,
            test_name=test_name,
            test_category=test_category,
            success=success,
            total_time=total_time,
            target_position=f"{target_pos[0]:.3f},{target_pos[1]:.3f},{target_pos[2]:.3f}",
            target_orientation=str(target_ori) if target_ori else "default",
            constraint_type=constraint_type,
            error_message=error_msg
        )
    
    def time_movement(self, movement_func, *args, **kwargs) -> Tuple[bool, float, str]:
        """
        Time a movement function.
        returns tuple of (success, , error_message)
        """
        try:
            start_time = time.time()
            
            # Clear any previous targets
            self.controller.arm_group.clear_pose_targets()
            
            # Execute the movement
            success = movement_func(*args, **kwargs)
            error_msg = "" if success else "Movement fail!"
            
            end_time = time.time()
            total_time = end_time - start_time
            
            # TODO: also implement planning time and execution time counters to improve bm (see report!!)
            # planning_time =
            # execution_time =
            
            return success, total_time, error_msg
            
        except Exception as e:
            error_msg = str(e)
            rospy.logerr(f"Movement failed with exception: {error_msg}")
            return False, 0.0, error_msg
    
    def benchmark_preset_poses(self, run_number: int):
        """Benchmark all preset poses."""
        rospy.loginfo(f"Run {run_number}: Benchmarking preset poses...")
        
        presets = self.controller.get_predefined_poses()
        
        for preset_name, preset_data in presets.items():
            rospy.loginfo(f"  Testing preset: {preset_name}")
            
            success, total_time, error = self.time_movement(
                self.controller.move_to_preset, preset_name
            )
            
            result = self.create_result(
                run_number=run_number,
                test_name=f"preset_{preset_name}",
                test_category="preset_poses",
                success=success,
                total_time=total_time,
                target_pos=preset_data['position'],
                target_ori=preset_data['rpy'],
                error_msg=error
            )
            
            self.results.append(result)
            rospy.sleep(PAUSE_BETWEEN_TESTS)   # NOTE: do a  pause between each movement
                                # TODO: try also without pauses
    
    def benchmark_pointing(self, run_number: int):
        """Benchmark pointing at specific positions."""
        rospy.loginfo(f"Run {run_number}: Benchmarking pointing...")
        
        targets = [
            ((0.8, 0.0, 1.2), "front"),
            ((0.5, 0.5, 1.0), "front_left"),
            ((0.5, -0.5, 1.0), "front_right"),
            ((0.3, 0.0, 1.8), "up"),
            ((0.3, 0.0, 0.6), "down")
        ]
        
        for target_pos, description in targets:
            rospy.loginfo(f"  Testing pointing at: {description}")
            
            success, total_time, error = self.time_movement(
                self.controller.point_at, target_pos
            )
            
            result = self.create_result(
                run_number=run_number,
                test_name=f"point_{description}",
                test_category="pointing",
                success=success,
                total_time=total_time,
                target_pos=target_pos,
                error_msg=error
            )
            
            self.results.append(result)
            rospy.sleep(PAUSE_BETWEEN_TESTS)
    
    def benchmark_player_pointing(self, run_number: int):
        """Benchmark pointing at players."""
        rospy.loginfo(f"[Run {run_number}] Benchmarking player pointing...")
        
        test_players = [
            (Player([], 0.0, (0, 0, 0, 0), 0), "front_0deg"),
            (Player([], 1.57, (0, 0, 0, 0), 1), "left_90deg"),
            (Player([], -1.57, (0, 0, 0, 0), 2), "right_neg90deg"),
            (Player([], 3.14, (0, 0, 0, 0), 3), "back_180deg"),
            (Player([], 0.785, (0, 0, 0, 0), 4), "front_left_45deg"),
            (Player([], -0.785, (0, 0, 0, 0), 5), "front_right_neg45deg"),
        ]
        
        for player, description in test_players:
            rospy.loginfo(f"  Testing player pointing: {description}")
            
            success, total_time, error = self.time_movement(
                self.controller.point_at_player, player
            )
            
            # Calculate expected arm position for logging
            arm_distance = 0.6
            arm_height = 1.2
            yaw = player.yaw
            arm_x = arm_distance * np.cos(yaw)
            arm_y = arm_distance * np.sin(yaw)
            
            result = self.create_result(
                run_number=run_number,
                test_name=f"player_{description}",
                test_category="player_pointing",
                success=success,
                total_time=total_time,
                target_pos=(arm_x, arm_y, arm_height),
                target_ori=(0, 0, yaw),
                error_msg=error
            )
            
            self.results.append(result)
            rospy.sleep(PAUSE_BETWEEN_TESTS)
    
    def benchmark_constrained_movements(self, run_number: int):
        """Benchmark movements with different constraints."""
        rospy.loginfo(f"[Run {run_number}] Benchmarking constrained movements...")
        
        test_player = Player([], 1.57, (0, 0, 0, 0), 1)  # Left (90 degrees)
        
        # Test without constraints
        rospy.loginfo("  Testing WITHOUT constraints...")
        self.controller.clear_constraints()
        success, total_time, error = self.time_movement(
            self.controller.point_at_player, test_player
        )
        
        result = self.create_result(
            run_number=run_number,
            test_name="constrained_none",
            test_category="constraints",
            success=success,
            total_time=total_time,
            target_pos=(0, 0.6, 1.2),
            constraint_type="none",
            error_msg=error
        )
        self.results.append(result)
        rospy.sleep(PAUSE_BETWEEN_TESTS)
        
        # Test with elbow constraint only
        rospy.loginfo("  Testing WITH elbow constraint...")
        self.controller.set_elbow_down_constraint(True, max_elbow_angle=-0.2)
        success, total_time, error = self.time_movement(
            self.controller.point_at_player, test_player
        )
        
        result = self.create_result(
            run_number=run_number,
            test_name="constrained_elbow",
            test_category="constraints",
            success=success,
            total_time=total_time,
            target_pos=(0, 0.6, 1.2),
            constraint_type="elbow_down",
            error_msg=error
        )
        self.results.append(result)
        rospy.sleep(PAUSE_BETWEEN_TESTS)
        
        # Test with shoulder constraint only
        rospy.loginfo("  Testing WITH shoulder constraint...")
        self.controller.clear_constraints()
        self.controller.set_shoulder_down_constraint(True, max_shoulder_angle=0.3)
        success, total_time, error = self.time_movement(
            self.controller.point_at_player, test_player
        )
        
        result = self.create_result(
            run_number=run_number,
            test_name="constrained_shoulder",
            test_category="constraints",
            success=success,
            total_time=total_time,
            target_pos=(0, 0.6, 1.2),
            constraint_type="shoulder_down",
            error_msg=error
        )
        self.results.append(result)
        rospy.sleep(PAUSE_BETWEEN_TESTS)
        
        # Test with natural pointing constraints
        rospy.loginfo("  Testing WITH natural pointing constraints...")
        self.controller.set_natural_pointing_constraints(True)
        success, total_time, error = self.time_movement(
            self.controller.point_at_player, test_player
        )
        
        result = self.create_result(
            run_number=run_number,
            test_name="constrained_natural",
            test_category="constraints",
            success=success,
            total_time=total_time,
            target_pos=(0, 0.6, 1.2),
            constraint_type="natural_pointing",
            error_msg=error
        )
        self.results.append(result)
        
        # Clean up constraints
        self.controller.clear_constraints()
        rospy.sleep(PAUSE_BETWEEN_TESTS)
    
    
    def benchmark_relative_movements(self, run_number: int):
        """Benchmark relative movements."""
        rospy.loginfo(f"Run {run_number}: Benchmarking relative movements...")
        
        # First move to a known position
        self.controller.move_to_preset('home')
        rospy.sleep(1)
        
        relative_moves = [
            ((0.1, 0.0, 0.0), (0.0, 0.0, 0.0), "forward"),
            ((0.0, 0.1, 0.0), (0.0, 0.0, 0.0), "left"),
            ((0.0, 0.0, 0.1), (0.0, 0.0, 0.0), "up"),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.3), "rotate_yaw"),
        ]
        
        for delta_pos, delta_rpy, description in relative_moves:
            rospy.loginfo(f"  Testing relative move: {description}")
            
            current_pose = self.controller.get_current_pose()
            target_pos = (
                current_pose.position.x + delta_pos[0],
                current_pose.position.y + delta_pos[1],
                current_pose.position.z + delta_pos[2]
            )
            
            success, total_time, error = self.time_movement(
                self.controller.move_relative, delta_pos, delta_rpy
            )
            
            result = self.create_result(
                run_number=run_number,
                test_name=f"relative_{description}",
                test_category="relative_movements",
                success=success,
                total_time=total_time,
                target_pos=target_pos,
                target_ori=delta_rpy,
                error_msg=error
            )
            
            self.results.append(result)
            rospy.sleep(PAUSE_BETWEEN_TESTS)
    
    def run_full_benchmark(self, num_runs: int = 10):
        """
        Run complete benchmark suite multiple times.
        num_runs: Number of times to run the complete benchmark
        """
        rospy.loginfo(f"Starting full benchmark suite ({num_runs} runs)...")
        start_time = time.time()
        
        for run in range(1, num_runs + 1):
            rospy.loginfo(f"BENCHMARK RUN {run} /{num_runs}")
            
            # Return to home position before each run
            self.controller.move_to_preset('home')
            rospy.sleep(PAUSE_BETWEEN_TESTS)
            
            # Run all benchmark categories
            try:
                self.benchmark_preset_poses(run)
                self.benchmark_pointing(run)
                self.benchmark_player_pointing(run)
                self.benchmark_constrained_movements(run)
                self.benchmark_relative_movements(run)
                
            except Exception as e:
                rospy.logerr(f"Error during run {run}: {e}")
                continue
            
            # Return to home after each run
            self.controller.move_to_preset('home')
            rospy.sleep(PAUSE_BETWEEN_TESTS)
            
            rospy.loginfo(f"Completed run {run}/{num_runs}")
        
        # log stuff
        total_time = time.time() - start_time
        rospy.loginfo(f"Benchmark completed!")
        rospy.loginfo(f"Total time: {total_time:.2f} seconds")
        rospy.loginfo(f"Total tests: {len(self.results)}")
    
    def save_results(self):
        """Save benchmark results to TSV file."""
        rospy.loginfo(f"Saving results to {self.output_file}...")
        
        if not self.results:
            rospy.logwarn("No results to save!")
            return
        
        # Get field names from the first result
        fieldnames = list(
            asdict(self.results[0]).keys()
        )
        
        with open(self.output_file, 'w', newline='') as tsvfile:
            writer = csv.DictWriter(tsvfile, fieldnames=fieldnames, delimiter='\t')
            writer.writeheader()
            
            for result in self.results:
                writer.writerow(asdict(result))
        
        rospy.loginfo(f"Results saved successfully!")
    
    def print_summary(self):
        """Print statistics of the benchmark (summary)."""
        if not self.results:
            rospy.logwarn("No results!")
            return
        
        total_tests = len(self.results)
        successful_tests = sum(1 for r in self.results if r.success)
        failed_tests = total_tests - successful_tests
        
        avg_total_time = np.mean([r.total_time for r in self.results if r.success])

        print("BENCHMARK SUMMARY")
        print(f"Total tests: {total_tests}")
        print(f"Successful: {successful_tests} ({100*successful_tests/total_tests}%)")
        print(f"Failed: {failed_tests} ({100*failed_tests/total_tests}%)")
        print(f"Average Total Time:  {avg_total_time} seconds")
        



if __name__ == '__main__':

    try:
        RB_BENCHMARK_OUTPUT_FILE = "src/benchmarks/robot_benchmark.tsv"

        # Create benchmark instance
        benchmark = ArmControllerBenchmark(output_file=RB_BENCHMARK_OUTPUT_FILE)   
        # Run full benchmarks
        benchmark.run_full_benchmark(num_runs=10)
   
        # Save results to file and print results
        benchmark.save_results()
        benchmark.print_summary()
        
        rospy.loginfo("Benchmark ok!")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Benchmark interrupted")
    except Exception as e:
        rospy.logerr(f"Benchmark failed with error: {e}")
        raise