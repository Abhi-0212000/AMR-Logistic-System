import numpy as np
import matplotlib.pyplot as plt
import os
import time
import pandas as pd
from typing import List, Dict, Optional
from dataclasses import dataclass

from window_manager import WindowManager, WindowConfig
from trajectory_optimizer import TrajectoryOptimizerConfig
from trajectory_blending import TrajectoryBlender
from velocity_profile import VelocityProfileVisualizer
from spline_generation import SplineVisualizer
from common_types import TrajectoryInfo, PlanningPoint

class TestWindowedPlanner:
    def __init__(self, save_dir: str = "trajectory_data"):
        """Initialize test environment with save directory for data/plots"""
        self.save_dir = save_dir
        self.plots_dir = os.path.join(save_dir, "plots")
        self.data_dir = os.path.join(save_dir, "data")
        
        # Create directories if they don't exist
        os.makedirs(self.plots_dir, exist_ok=True)
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Initialize storage for trajectory data
        self.trajectories = []
        self.window_stats = []
        
    def save_trajectory_data(self, trajectory: TrajectoryInfo, window_index: int):
        """Save trajectory data and plots for a window"""
        # Save planning points data
        points_data = []
        for point in trajectory.planning_points:
            point_dict = {
                'x': point.position[0],
                'y': point.position[1],
                'velocity': point.velocity,
                'curvature': point.curvature,
                'heading': point.heading,
                'time': point.time,
                'arc_length': point.arc_length
            }
            points_data.append(point_dict)
            
        df = pd.DataFrame(points_data)
        df.to_csv(os.path.join(self.data_dir, f'window_{window_index}_data.csv'), index=False)
        print(f"Saved trajectory data to {os.path.join(self.data_dir, f'window_{window_index}_data.csv')}")
        
        # Save visualization plots
        plt.figure(figsize=(15, 10))
        
        # Plot trajectory
        plt.subplot(2, 2, 1)
        plt.plot([p.position[0] for p in trajectory.planning_points],
                [p.position[1] for p in trajectory.planning_points], 'b-', label='Path')
        plt.plot([w[0] for w in trajectory.waypoints],
                [w[1] for w in trajectory.waypoints], 'ro', label='Waypoints')
        plt.title(f'Window {window_index} - Trajectory')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        
        # Plot velocity profile
        plt.subplot(2, 2, 2)
        arc_lengths = [p.arc_length for p in trajectory.planning_points]
        velocities = [p.velocity for p in trajectory.planning_points]
        plt.plot(arc_lengths, velocities, 'g-')
        plt.title('Velocity Profile')
        plt.xlabel('Arc Length (m)')
        plt.ylabel('Velocity (m/s)')
        plt.grid(True)
        
        # Plot curvature
        plt.subplot(2, 2, 3)
        curvatures = [p.curvature for p in trajectory.planning_points]
        plt.plot(arc_lengths, curvatures, 'r-')
        plt.title('Curvature Profile')
        plt.xlabel('Arc Length (m)')
        plt.ylabel('Curvature (1/m)')
        plt.grid(True)
        
        # Plot time history
        plt.subplot(2, 2, 4)
        times = [p.time for p in trajectory.planning_points]
        plt.plot(times, velocities, 'b-')
        plt.title('Velocity vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.grid(True)
        
        plt.tight_layout()
        plot_path = os.path.join(self.plots_dir, f'window_{window_index}_plots.png')
        plt.savefig(plot_path)
        plt.close()
        print(f"Saved plots to {plot_path}")
        
    def run_test(self, lanelet_dict: Dict, lanelet_ids: List[int]):
        """Run windowed trajectory planner test"""
        # Configure window manager
        window_config = WindowConfig(
            planning_time=1.1,    # Time needed for planning (sec)
            buffer_time=0.3,      # Extra buffer time (sec)
            lookahead_points=6,   # Points to look ahead (~6m with 1m spacing)
            window_spacing=2.0    # Approx spacing between points (meters)
        )
        
        # Initialize window manager
        window_manager = WindowManager(lanelet_dict, lanelet_ids, window_config)
        
        # Simulation parameters
        control_rate = 100  # Hz
        rate_period = 1.0/control_rate
        sim_start_time = time.time()
        current_time = 0.0
        window_index = 0
        
        # Initial robot position (first waypoint)
        robot_position = np.array(window_manager.centerline_processor.processed_points[0])
        
        print("\nStarting windowed trajectory planning test...")
        print(f"Control rate: {control_rate} Hz (period: {rate_period:.3f}s)")
        print(f"Save directory: {self.save_dir}\n")
        
        while True:
            loop_start = time.time()
            
            # Check if we need new trajectory
            if (window_manager.next_planning_time is None or 
                current_time >= window_manager.next_planning_time):
                
                print(f"\nProcessing window {window_index} at time {current_time:.2f}s")
                print(f"Robot position: [{robot_position[0]:.2f}, {robot_position[1]:.2f}]")
                
                # Process window
                trajectory = window_manager.process_window(robot_position)
                
                if trajectory and trajectory.planning_points:
                    print(f"Window {window_index} generated successfully:")
                    print(f"Number of points: {len(trajectory.planning_points)}")
                    print(f"Window duration: {trajectory.planning_points[-1].time:.2f}s")
                    
                    # Save trajectory data and plots
                    self.save_trajectory_data(trajectory, window_index)
                    
                    # Store stats
                    self.window_stats.append({
                        'window_index': window_index,
                        'sim_time': current_time,
                        'num_points': len(trajectory.planning_points),
                        'duration': trajectory.planning_points[-1].time,
                        'start_velocity': trajectory.planning_points[0].velocity,
                        'end_velocity': trajectory.planning_points[-1].velocity,
                        'max_velocity': max(p.velocity for p in trajectory.planning_points),
                        'mean_velocity': np.mean([p.velocity for p in trajectory.planning_points]),
                        'total_distance': trajectory.planning_points[-1].arc_length
                    })
                    
                    # Here we would publish trajectory in ROS environment
                    # trajectory_msg = convert_to_trajectory_msg(trajectory)
                    # trajectory_publisher.publish(trajectory_msg)
                    
                    # Calculate window timing
                    window_start_time = trajectory.planning_points[0].time
                    window_end_time = trajectory.planning_points[-1].time
                    window_duration = window_end_time - window_start_time
                    
                    # Update next planning time
                    window_manager.next_planning_time = (
                        current_time + 
                        window_duration - 
                        (window_manager.config.planning_time + window_manager.config.buffer_time)
                    )
                    print(f"Next planning time set to: {window_manager.next_planning_time:.2f}s")
                    
                    # Calculate join point time in trajectory time frame
                    current_traj_time_of_join_point = current_time + window_manager.next_planning_time
                    
                    # Find corresponding point on trajectory for next window's start position
                    join_point_found = False
                    for i, point in enumerate(trajectory.planning_points):
                        if point.time >= current_traj_time_of_join_point:
                            robot_position = point.position
                            join_point_found = True
                            break
                    
                    if not join_point_found:
                        robot_position = trajectory.planning_points[-1].position
                    
                    print(f"Current time: {current_time:.2f}")
                    print(f"Join point time for next window: {current_traj_time_of_join_point:.2f}")
                    print(f"Robot position updated to: [{robot_position[0]:.2f}, {robot_position[1]:.2f}]")
                    
                    window_index += 1
                    
                    # If this is the last window, wait for its completion and exit
                    if window_manager.current_window_is_last:
                        print("\nExecuting final window - waiting for completion...")
                        sleep_time = window_duration  # Sleep for window traversal time
                        if sleep_time > 0:
                            print(f"Sleeping for {sleep_time:.2f}s to complete final window")
                            time.sleep(sleep_time)
                        print("\nGoal reached - planning complete!")
                        break
                    else:
                        # Not last window - sleep until next window should start
                        sleep_time = window_manager.next_planning_time - current_time
                        if sleep_time > 0:
                            print(f"Sleeping for {sleep_time:.2f}s until next window")
                            time.sleep(sleep_time)
                            # Update current time to when next window should start
                            current_time = window_manager.next_planning_time
                
                else:
                    print("Failed to generate trajectory")
                    break
            
            # Calculate sleep time to maintain control rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, rate_period - elapsed)
            time.sleep(sleep_time)
            
            # Update simulation time
            current_time += rate_period
        
        # Save overall statistics
        stats_df = pd.DataFrame(self.window_stats)
        stats_path = os.path.join(self.data_dir, 'overall_statistics.csv')
        stats_df.to_csv(stats_path, index=False)
        print(f"\nSaved overall statistics to {stats_path}")
        
        print("\nTest completed!")
        print(f"Total windows processed: {window_index}")
        print(f"Total simulation time: {current_time:.2f} seconds")
        print(f"Actual runtime: {time.time() - sim_start_time:.2f} seconds")
        print(f"Data saved to: {self.save_dir}")

def main():
    # Example lanelet data (same as in local_planner_example.py)
    lanelet_dict = {
        -6995: {
            "length": 1.87,
            "centerline_points": [
                [19.0998, 4.24597], [17.6539, 5.35312]
            ],
            "left_boundary": np.array([ [18.504, 3.38053], [16.979, 4.46012] ]),
            "right_boundary": np.array([ [19.7237, 5.15215], [18.3057, 6.21025] ])
        },
        -7019: {
            "length": 3.19,
            "centerline_points": [
                [17.6539, 5.35312], [16.9326, 5.90554], [16.2295, 6.43471], [15.7666, 6.79154], [15.1116, 7.28624]
            ],
            "left_boundary": np.array([ [16.979, 4.46012], [14.454, 6.4172] ]),
            "right_boundary": np.array([ [18.3057, 6.21025], [15.7474, 8.06041] ])
        },
        -6996: {
            "length": 3.06,
            "centerline_points": [
                [15.1116, 7.28624], [14.4443, 7.80432], [13.7414, 8.32168], [12.8583, 8.97716]
            ],
            "left_boundary": np.array([ [14.454, 6.4172], [13.5285, 7.11906], [12.736, 7.68188], [11.991, 8.23383] ]),
            "right_boundary": np.array([ [15.7474, 8.06041], [14.6891, 8.88951], [14.0755, 9.38503], [13.389, 9.97357] ])
        },
        -6999: {
            "length": 5.42,
            "centerline_points": [
                [12.8583, 8.97716], [12.2636, 9.41402], [11.6704, 9.77417], [11.2832, 9.89637], [10.5473, 9.99393], [9.97879, 9.94867], [9.35417, 9.864], [8.4243, 9.5917], [7.62356, 9.37511], [6.6296, 8.75324], [5.98117, 8.02015], [5.65965, 7.51788], [5.36672, 7.0406]
            ],
            "left_boundary": np.array([ [11.991, 8.23383], [11.7149, 8.42907], [11.2115, 8.73787], [10.7119, 8.85784], [10.3261, 8.61404], [10.0488, 8.26614], [9.81941, 7.89558], [9.50831, 7.46436], [9.17449, 6.98546], [8.78196, 6.48178], [8.5516, 6.15843], [8.25141, 5.77465] ]),
            
            "right_boundary": np.array([ [13.389, 9.97357], [13.0033, 10.3201], [12.5475, 10.6181], [12.2008, 10.7883], [11.995, 10.872], [11.5086, 11.0697], [11.0918, 11.1913], [10.2724, 11.3168], [9.703, 11.3763], [9.1809, 11.4368], [8.44671, 11.4458], [7.78494, 11.3855], [7.06586, 11.2295], [6.52384, 11.1007], [6.12499, 10.9157], [5.80921, 10.7205], [5.4004, 10.4409], [5.09669, 10.2341], [4.66492, 9.91857], [4.32653, 9.66981], [4.04825, 9.37502], [3.79223, 9.15151], [3.43345, 8.80946] ])
        },
        -6985: {
            "length": 20.10,
            "centerline_points": [
                [5.36672, 7.0406], [4.8454, 6.19118], [3.2809, 3.89322], [2.01286, 2.16787], [0.723955, 0.300428], [-1.12082, -2.192], [-3.33258, -5.28206], [-4.4865, -6.79262], [-6.01396, -8.57034]
            ],
            "left_boundary": np.array([ [8.25141, 5.77465], [-3.60881, -10.4589] ]),
            "right_boundary": np.array([ [3.43345, 8.80946], [-8.4099, -7.15374] ])
        },
        -7011: {
            "length": 17.30,
            "centerline_points": [
                [-6.01396, -8.57034], [-7.19296, -10.0106], [-8.23884, -10.9995], [-9.38688, -11.6126], [-11.1503, -12.2379], [-11.944, -12.4708], [-12.8729, -12.5318], [-13.6901, -12.56], [-15.1697, -12.5302], [-16.9741, -12.2826]
            ],
            "left_boundary": np.array([ [-3.60881, -10.4589], [-4.86529, -11.5701], [-5.72838, -12.2248], [-6.78315, -12.7653], [-7.97994, -13.3085], [-9.34334, -13.8078], [-10.7096, -14.1655], [-11.8888, -14.4014], [-13.2611, -14.4523], [-14.0646, -14.5272], [-15.2951, -14.5634], [-16.2532, -14.6061], [-17.2705, -14.6499], [-18.1228, -14.655], [-18.6563, -14.6183], [-19.3209, -14.537], [-19.843, -14.4765] ]),
            "right_boundary": np.array([ [-8.4099, -7.15374], [-9.97814, -9.26286], [-11.1396, -9.79368], [-12.6838, -10.1313], [-13.5649, -10.1724], [-14.6982, -10.0414] ])
        },
        -7010: {
            "length": 9.57,
            "centerline_points": [
                [-16.9741, -12.2826], [-17.8822, -11.8637], [-18.5469, -11.1803], [-18.9396, -10.4797], [-19.1558, -9.72833], [-19.1452, -9.06693], [-19.088, -8.69983], [-19.0182, -8.30878], [-18.876, -7.71561], [-18.7455, -7.32025], [-18.5185, -6.94107], [-18.247, -6.59919], [-18.1226, -6.44341]
            ],
            "left_boundary": np.array([ [-19.843, -14.4765], [-20.0488, -14.2445], [-20.6685, -13.4421], [-21.0998, -12.5887], [-21.2747, -11.5296], [-21.1634, -10.5769], [-21.001, -9.81215], [-20.8453, -9.30137], [-20.4942, -8.50334], [-20.1515, -7.87078], [-19.8544, -7.33357], [-19.5098, -6.80134], [-19.2798, -6.46028], [-19.0833, -6.22024], [-18.8617, -5.91098] ]),
            "right_boundary": np.array([ [-14.6982, -10.0414], [-15.5572, -9.70419], [-16.1924, -9.31535], [-16.589, -9.01624], [-16.7945, -8.80112], [-16.8765, -8.71528], [-17.0159, -8.56936], [-17.1365, -8.44306], [-17.2571, -8.31679], [-17.3749, -8.19354], [-17.5994, -7.91463], [-17.6912, -7.76296], [-17.695, -7.57412], [-17.6326, -7.4371], [-17.5761, -7.3002], [-17.3544, -6.93731] ])
        },
        -6994: {
            "length": 11.06,
            "centerline_points": [
                [-18.1226, -6.44341], [-17.2651, -5.3695], [-16.446, -4.29063], [-15.6737, -3.23631], [-14.7634, -1.97852], [-13.8303, -0.673066], [-12.8511, 0.692344], [-11.9415, 1.98553], [-11.8437, 2.12914], [-11.7293, 2.32918], [-11.6577, 2.4633]
            ],
            "left_boundary": np.array([ [-18.8617, -5.91098], [-12.341, 3.02842] ]), # Original left boundary: [-12.341, 3.02842], [-18.8617, -5.91098]
            "right_boundary": np.array([ [-17.3544, -6.93731], [-10.9541, 1.74536] ]) # Original right boundary: [-10.9541, 1.74536], [-17.3544, -6.93731]
        },
        -7001: {
            "length": 2.42,
            "centerline_points": [
                [-11.6577, 2.4633], [-11.4953, 2.76773], [-11.3558, 3.19554], [-11.2719, 3.74032], [-11.2616, 3.97076], [-11.2662, 4.2009], [-11.3436, 4.37057], [-11.4012, 4.59375], [-11.4799, 4.82833], [-11.5845, 5.02698], [-11.7932, 5.25602], [-12.0189, 5.44046]
            ],
            "left_boundary": np.array([ [-12.341, 3.02842], [-11.943, 3.85099], [-11.7971, 4.25532], [-11.8741, 4.55781], [-12.3797, 5.12927] ]),
            "right_boundary": np.array([ [-10.9541, 1.74536], [-10.74, 2.29273], [-10.6227, 2.94443], [-10.6393, 3.78239], [-10.7374, 4.25272], [-10.9583, 4.94495], [-11.3114, 5.43384], [-11.8656, 5.91873] ])
        },
        -6993: {
            "length": 1.82,
            "centerline_points": [
                [-12.0189, 5.44046], [-12.2932, 5.69477], [-12.5905, 5.91025], [-12.8963, 6.1049], [-13.1033, 6.24543], [-13.3877, 6.40804], [-13.6214, 6.57407]
            ],
            "left_boundary": np.array([ [-12.3797, 5.12927], [-12.6472, 5.3365], [-13.32, 5.83085], [-13.8598, 6.18616] ]),
            "right_boundary": np.array([ [-11.8656, 5.91873], [-12.3102, 6.25231], [-12.7781, 6.56182], [-13.3896, 6.95113] ])
        }
        
    }


    lanelet_ids = [-6995, -7019, -6996, -6999, -6985, -7011, -7010, -6994, -7001, -6993]
    
    # lanelet_dict = {
    #     -6995: {
    #         "length": 1.87,
    #         "centerline_points": [
    #             [19.0998, 4.24597], [17.6539, 5.35312]
    #         ],
    #         "left_boundary": np.array([ [18.504, 3.38053], [16.979, 4.46012] ]),
    #         "right_boundary": np.array([ [19.7237, 5.15215], [18.3057, 6.21025] ])
    #     },
    #     -7019: {
    #         "length": 3.19,
    #         "centerline_points": [
    #             [17.6539, 5.35312], [16.9326, 5.90554], [16.2295, 6.43471], 
    #             [15.7666, 6.79154], [15.1116, 7.28624]
    #         ],
    #         "left_boundary": np.array([ [16.979, 4.46012], [14.454, 6.4172] ]),
    #         "right_boundary": np.array([ [18.3057, 6.21025], [15.7474, 8.06041] ])
    #     }
        # -6996: {
        #     "length": 3.06,
        #     "centerline_points": [
        #         [15.1116, 7.28624], [14.4443, 7.80432], [13.7414, 8.32168], [12.8583, 8.97716]
        #     ],
        #     "left_boundary": np.array([ [14.454, 6.4172], [13.5285, 7.11906], 
        #                               [12.736, 7.68188], [11.991, 8.23383] ]),
        #     "right_boundary": np.array([ [15.7474, 8.06041], [14.6891, 8.88951], 
        #                                [14.0755, 9.38503], [13.389, 9.97357] ])
        # },
        # -6999: {
        #     "length": 5.42,
        #     "centerline_points": [
        #         [12.8583, 8.97716], [12.2636, 9.41402], [11.6704, 9.77417], 
        #         [11.2832, 9.89637], [10.5473, 9.99393], [9.97879, 9.94867], 
        #         [9.35417, 9.864], [8.4243, 9.5917]
        #     ],
        #     "left_boundary": np.array([ [11.991, 8.23383], [11.7149, 8.42907], 
        #                               [11.2115, 8.73787], [10.7119, 8.85784],
        #                               [10.3261, 8.61404], [10.0488, 8.26614], 
        #                               [9.81941, 7.89558], [9.50831, 7.46436],
        #                               [9.17449, 6.98546], [8.78196, 6.48178], 
        #                               [8.5516, 6.15843], [8.25141, 5.77465] ]),
        #     "right_boundary": np.array([ [13.389, 9.97357], [13.0033, 10.3201], 
        #                                [12.5475, 10.6181], [12.2008, 10.7883],
        #                                [11.995, 10.872], [11.5086, 11.0697], 
        #                                [11.0918, 11.1913], [10.2724, 11.3168],
        #                                [9.703, 11.3763], [9.1809, 11.4368], 
        #                                [8.44671, 11.4458], [7.78494, 11.3855] ])
        # }
    # }

    # lanelet_ids = [-6995, -7019]
    
    # Create and run test
    test = TestWindowedPlanner(save_dir="windowed_planner_results")
    test.run_test(lanelet_dict, lanelet_ids)

if __name__ == "__main__":
    main()