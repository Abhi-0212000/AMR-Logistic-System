
# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.patches import Rectangle
# import os
# import pandas as pd
# import re
# import glob
# from typing import List, Dict, Tuple, Optional

# class TrajectoryAnimator:
#     """
#     Class for creating animations of trajectory data from multiple windows.
#     Handles trajectory blending and visualization of robot movement.
#     """
    
#     def __init__(self, base_dir: str = None, num_windows: int = 2):
#         """
#         Initialize animator with configuration.
        
#         Args:
#             base_dir: Base directory for trajectory data
#             num_windows: Number of windows to load for animation
#         """
#         # Set up data directory
#         self.data_dir = "/home/abhi/ros2_lanelet2_ws/trajectory_data/data" if base_dir is None else base_dir
#         print(f"Using data directory: {self.data_dir}")
        
#         # Animation settings
#         self.fps = 30
#         self.frame_interval = 1000 / self.fps  # milliseconds
#         self.robot_size = (0.5, 0.3)  # width, height in meters
        
#         # Load trajectory windows
#         self.windows = self._load_windows(num_windows)
        
#         # Initialize plot
#         self.fig, self.ax = plt.subplots(figsize=(12, 8))
#         self.ax.set_aspect('equal')
        
#         # Initialize text displays
#         self._init_text_displays()
        
#         # Initialize robot marker
#         self.robot = Rectangle((0, 0), self.robot_size[0], self.robot_size[1], 
#                              fc='red', alpha=0.7)
#         self.ax.add_patch(self.robot)
        
#         # Window tracking
#         self.current_window_data = None
#         self.next_window_data = None
#         self.time_offset = 0  # Initialize time offset
#         self.current_frame_time = 0 # Initialize the time for the current frame
#         self.time_step = 0.02  # Time increment for each frame (adjust as needed)
        
#         # initialize a flag to plot next window
#         self.plotted_next_window = False
#         self.current_traj_line = None
#         self.next_traj_line = None

#     def _init_text_displays(self):
#         """Initialize text display elements on plot."""
#         self.time_text = self.ax.text(0.4, 1.02, '', 
#                                     transform=self.ax.transAxes, 
#                                     ha='center')
#         self.window_text = self.ax.text(0.02, 1.02, '', 
#                                       transform=self.ax.transAxes)
#         self.velocity_text = self.ax.text(0, 0, '', visible=False)
    
#     def _load_windows(self, num_windows: int) -> List[Dict]:
#         """
#         Load specified number of trajectory windows.
        
#         Args:
#             num_windows: Number of windows to load
            
#         Returns:
#             List of dictionaries containing window data
#         """
#         windows = []
        
#         # Get sorted window directories
#         window_dirs = sorted(
#             glob.glob(os.path.join(self.data_dir, 'window_*_time_*')),
#             key=lambda x: int(re.search(r'window_(\d+)', x).group(1))
#         )
#         window_dirs = window_dirs[:num_windows]
        
#         for window_dir in window_dirs:
#             try:
#                 # Extract window index
#                 idx_match = re.search(r'window_(\d+)', window_dir)
#                 window_idx = int(idx_match.group(1))
                
#                 # Load trajectory data
#                 window_data = {
#                     'index': window_idx,
#                     'initial_traj': pd.read_csv(os.path.join(window_dir, 
#                         f'initial_trajectory_{window_idx}.csv')),
#                     'optimized_traj': pd.read_csv(os.path.join(window_dir,
#                         f'optimized_trajectory_{window_idx}.csv'))
#                 }
                
#                 # Load boundaries
#                 boundaries = np.load(os.path.join(window_dir, 
#                     f'boundaries_{window_idx}.npz'))
#                 window_data.update({
#                     'left_boundary': boundaries['left_boundary'],
#                     'right_boundary': boundaries['right_boundary']
#                 })
                
#                 windows.append(window_data)
                
#             except Exception as e:
#                 print(f"Error loading window {window_idx}: {str(e)}")
#                 continue
        
#         print(f"Loaded {len(windows)} trajectory windows")
#         return windows

#     def _find_join_point(self) -> Tuple[Optional[int], Optional[float]]:
#         """
#         Find join point between current and next trajectory.
        
#         Returns:
#             Tuple of (join_index, join_time) or (None, None) if not found
#         """
#         if not self.next_window_data:
#             return None, None
            
#         # Get first point of next trajectory
#         next_traj = self.next_window_data['optimized_traj']
#         join_point = next_traj.iloc[0]
        
#         # Find matching point in current trajectory
#         curr_traj = self.current_window_data['optimized_traj']
#         join_idx = None
#         min_dist = float('inf')
        
#         # Search last 25% of current trajectory
#         start_idx = int(len(curr_traj) * 0.75)
#         for i in range(start_idx, len(curr_traj)):
#             curr_point = curr_traj.iloc[i]
#             dist = np.sqrt((curr_point.x - join_point.x)**2 + 
#                          (curr_point.y - join_point.y)**2)
#             if dist < min_dist:
#                 min_dist = dist
#                 join_idx = i
        
#         # Get the time of the join point in the current trajectory
#         join_time_curr = curr_traj.iloc[join_idx]['time'] if join_idx is not None else None

#         # Get the time of the join point in the next trajectory (time at index 0)
#         join_time_next = next_traj.iloc[0]['time'] if next_traj is not None else None
                
#         return join_idx, join_time_curr, join_time_next
    
#     def _update_robot_marker(self, x: float, y: float, heading: float, velocity: float):
#         """
#         Update robot marker position and orientation.
        
#         Args:
#             x, y: Position coordinates
#             heading: Robot heading in radians
#             velocity: Current velocity
#         """
#         # Center robot on trajectory point
#         center_x = x - (self.robot_size[0]/2 * np.cos(heading) - 
#                        self.robot_size[1]/2 * np.sin(heading))
#         center_y = y - (self.robot_size[0]/2 * np.sin(heading) + 
#                        self.robot_size[1]/2 * np.cos(heading))
        
#         # Update robot position and orientation
#         self.robot.set_xy((center_x, center_y))
#         self.robot.angle = np.degrees(heading)
        
#         # Update velocity text position
#         if hasattr(self, 'velocity_text'):
#             self.velocity_text.remove()
#         self.velocity_text = self.ax.text(
#             x, y + 0.5, 
#             f'{velocity:.2f} m/s',
#             ha='center', 
#             va='bottom'
#         )

#     def init_animation(self):
#         """Initialize animation state."""
#         # Plot boundaries
#         for window in self.windows:
#             self.ax.plot(window['left_boundary'][:,0], 
#                         window['left_boundary'][:,1], 
#                         'k-', linewidth=2, alpha=0.8)
#             self.ax.plot(window['right_boundary'][:,0], 
#                         window['right_boundary'][:,1], 
#                         'k-', linewidth=2, alpha=0.8)
        
#         # Initialize windows
#         self.current_window_data = self.windows[0]
#         if len(self.windows) > 1:
#             self.next_window_data = self.windows[1]
        
#         # Calculate axis limits
#         self._set_axis_limits()
        
#         # Plot start and goal
#         self._plot_start_goal_points()
        
#         self.ax.grid(True)
#         self.ax.legend()
        
#         return self.robot, self.time_text, self.velocity_text, self.window_text
    
#     def _set_axis_limits(self):
#         """Calculate and set axis limits from all trajectories."""
#         x_coords = []
#         y_coords = []
#         for window in self.windows:
#             traj = window['optimized_traj']
#             x_coords.extend([traj.x.min(), traj.x.max()])
#             y_coords.extend([traj.y.min(), traj.y.max()])
            
#             # Add boundary points
#             x_coords.extend([
#                 window['left_boundary'][:,0].min(), 
#                 window['left_boundary'][:,0].max(),
#                 window['right_boundary'][:,0].min(), 
#                 window['right_boundary'][:,0].max()
#             ])
#             y_coords.extend([
#                 window['left_boundary'][:,1].min(), 
#                 window['left_boundary'][:,1].max(),
#                 window['right_boundary'][:,1].min(), 
#                 window['right_boundary'][:,1].max()
#             ])
        
#         margin = 2.0
#         self.ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
#         self.ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)
    
#     def _plot_start_goal_points(self):
#         """Plot start and goal points."""
#         start = self.windows[0]['optimized_traj'].iloc[0]
#         goal = self.windows[-1]['optimized_traj'].iloc[-1]
        
#         self.ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
#         self.ax.plot(goal.x, goal.y, 'ro', markersize=10, label='Goal')

#     def update_frame(self, frame: int):
#         """
#         Update animation frame based on trajectory time.
#         """
#         # Update the current frame time
#         self.current_frame_time += self.time_step

#         # Adjust current time with offset
#         adjusted_time = self.current_frame_time

#         # Get current trajectory and point
#         current_traj = self.current_window_data['optimized_traj']

#         # Check if we need to switch windows
#         if adjusted_time > current_traj['time'].iloc[-1] + self.time_offset:
#             if self.next_window_data:
#                 self.time_offset += current_traj['time'].iloc[-1]
#                 adjusted_time = self.current_frame_time 
#                 self.current_window_data = self.next_window_data
#                 current_traj = self.current_window_data['optimized_traj']

#                 window_index = self.windows.index(self.current_window_data)
#                 if window_index + 1 < len(self.windows):
#                     self.next_window_data = self.windows[window_index + 1]
#                 else:
#                     self.next_window_data = None

#                 print(f"Switched to window {self.current_window_data['index']} at adjusted_time: {adjusted_time:.2f}, offset: {self.time_offset:.2f}")
#                 self.plotted_next_window = False
#             else:
#                 # No more windows, stop updating
#                 return self.robot, self.time_text, self.velocity_text, self.window_text

#         # Find the closest point based on adjusted time
#         if self.current_window_data == self.windows[0]:
#            # Use 'time' column for the first window
#             time_diffs = np.abs(current_traj['time'] - (adjusted_time))
#         else:
#            # Offset 'time' for subsequent windows
#             time_diffs = np.abs(current_traj['time'] - (adjusted_time - self.time_offset))
        
        
#         point_idx = np.argmin(time_diffs)
#         point = current_traj.iloc[point_idx]
        
#         # Handle Trajectory Blending
#         if self.next_window_data:
#             join_idx, join_time_curr, join_time_next = self._find_join_point()
#             if join_idx is not None:
#                 # Check if the current time is close to the join time
#                 if (self.current_window_data == self.windows[0] and
#                         adjusted_time >= join_time_curr - self.time_step):
#                     if not self.plotted_next_window:
#                         # Plot the next trajectory (only once)
#                         next_traj = self.next_window_data['optimized_traj']
                        
#                         if self.next_traj_line:
#                             self.next_traj_line.remove()

#                         self.next_traj_line, = self.ax.plot(next_traj.x, next_traj.y, 'b-', alpha=0.3)
#                         self.plotted_next_window = True  # Update the flag
                        
#                         # Mark join point
#                         join_point = current_traj.iloc[join_idx]
#                         self.ax.plot(join_point.x, join_point.y, 'yo', markersize=8)

#                 if (adjusted_time >= self.time_offset + join_time_next) and (self.current_window_data == self.windows[0]):
#                     # Switch to the next trajectory
#                     self.current_window_data = self.next_window_data
#                     current_traj = self.current_window_data['optimized_traj']
#                     window_index = self.windows.index(self.current_window_data)

#                     if window_index + 1 < len(self.windows):
#                         self.next_window_data = self.windows[window_index + 1]
#                     else:
#                         self.next_window_data = None

#                     self.time_offset = self.current_frame_time - join_time_next # Update time offset
#                     adjusted_time = join_time_next
#                     self.plotted_next_window = False
#                     print(f"Switched to window {self.current_window_data['index']} at adjusted_time: {adjusted_time:.2f}, offset: {self.time_offset:.2f}")

#                     # Update the plot for the new current trajectory
#                     if self.current_traj_line:
#                         self.current_traj_line.remove()
#                     self.current_traj_line, = self.ax.plot(
#                         current_traj.x, current_traj.y, 'b-', linewidth=2
#                     )

#                     # Remove the next trajectory line if it exists
#                     if self.next_traj_line:
#                         self.next_traj_line.remove()
#                         self.next_traj_line = None

#                     # Find the closest point based on adjusted time
#                     time_diffs = np.abs(current_traj['time'] - (adjusted_time))
#                     point_idx = np.argmin(time_diffs)
#                     point = current_traj.iloc[point_idx]

#         # Update robot and text
#         self._update_robot_marker(point.x, point.y, point.heading, point.velocity)
#         self._update_text_displays(adjusted_time)

#         # Update trajectory visualization
#         self._update_trajectory_visualization(point_idx, point)

#         return self.robot, self.time_text, self.velocity_text, self.window_text

#     def _update_text_displays(self, adjusted_time):
#         """Update text displays with current state."""
#         self.time_text.set_text(f'Time: {adjusted_time:.2f}s')
#         self.window_text.set_text(
#             f'Window: {self.current_window_data["index"]}'
#         )
    
#     def _update_trajectory_visualization(self, point_idx: int, point):
#         """Update trajectory visualization with blending."""
        
#         # Plot current trajectory
#         current_traj = self.current_window_data['optimized_traj']
        
#         if self.current_traj_line:
#             self.current_traj_line.remove()

#         self.current_traj_line, = self.ax.plot(current_traj.x, current_traj.y, 'b-', linewidth=2)

#     def create_animation(self, output_file: str = 'trajectory_animation.mp4'):
#         """
#         Create and save animation.
        
#         Args:
#             output_file: Output video file path
#         """
#         # Calculate total animation time based on last window's last time entry
#         total_time = self.windows[-1]['optimized_traj']['time'].iloc[-1] + self.time_offset
#         num_frames = int(total_time / self.time_step)

#         print(f"Creating animation with {num_frames} frames "
#               f"for {total_time:.2f} seconds...")
        
#         # Create animation
#         anim = animation.FuncAnimation(
#             self.fig, 
#             self.update_frame,
#             init_func=self.init_animation,
#             frames=num_frames,
#             interval=self.frame_interval,
#             blit=True
#         )
        
#         # Save animation
#         print(f"Saving animation to {output_file}...")
#         writer = animation.FFMpegWriter(fps=self.fps)
#         anim.save(output_file, writer=writer)
#         print("Animation saved successfully!")

# def main():
#     """Main entry point."""
#     animator = TrajectoryAnimator(num_windows=2)
#     animator.create_animation()

# if __name__ == "__main__":
#     main()


# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.patches import Rectangle
# import os
# import pandas as pd
# import re
# import glob
# from typing import List, Dict, Tuple, Optional

# class TrajectoryAnimator:
#     """
#     Class for creating animations of trajectory data from multiple windows.
#     Handles trajectory blending and visualization of robot movement.
#     """

#     def __init__(self, base_dir: str = None, num_windows: int = None):
#         """
#         Initialize animator with configuration.

#         Args:
#             base_dir: Base directory for trajectory data
#             num_windows: Number of windows to load for animation
#         """
#         # Set up data directory
#         self.data_dir = "/home/abhi/ros2_lanelet2_ws/trajectory_data/data" if base_dir is None else base_dir
#         print(f"Using data directory: {self.data_dir}")

#         # Animation settings
#         self.fps = 30
#         self.frame_interval = 1000 / self.fps  # milliseconds
#         self.robot_size = (0.5, 0.3)  # width, height in meters

#         # Load trajectory windows
#         if num_windows is None:
#             # Get the number of window directories
#             num_windows = len(glob.glob(os.path.join(self.data_dir, 'window_*_time_*')))
#         self.windows = self._load_windows(num_windows)

#         # Initialize plot
#         self.fig, self.ax = plt.subplots(figsize=(12, 8))
#         self.ax.set_aspect('equal')

#         # Initialize text displays
#         self._init_text_displays()

#         # Initialize robot marker
#         self.robot = Rectangle((0, 0), self.robot_size[0], self.robot_size[1],
#                                fc='#FF5733',  # Use a nice orange color
#                                alpha=0.7)
#         self.ax.add_patch(self.robot)

#         # Window tracking
#         self.current_window_data = None
#         self.next_window_data = None
#         self.time_offset = 0  # Initialize time offset
#         self.current_frame_time = 0  # Initialize the time for the current frame
#         self.time_step = 0.02  # Time increment for each frame (adjust as needed)

#         # initialize a flag to plot next window
#         self.plotted_next_window = False
#         self.current_traj_line = None
#         self.next_traj_line = None

#         # Traversed path
#         self.traversed_path_line = None

#         # Join point marker
#         self.join_point_marker = None

#         # Initial trajectory lines for each window
#         self.initial_traj_lines = {}

#         # Store optimized trajectories for all windows
#         self.optimized_trajectories = {}

#     def _init_text_displays(self):
#         """Initialize text display elements on plot."""
#         self.time_text = self.ax.text(0.4, 1.02, '',
#                                       transform=self.ax.transAxes,
#                                       ha='center')
#         self.window_text = self.ax.text(0.02, 1.02, '',
#                                         transform=self.ax.transAxes)
#         self.velocity_text = self.ax.text(0, 0, '', visible=False)

#     def _load_windows(self, num_windows: int) -> List[Dict]:
#         """
#         Load specified number of trajectory windows.

#         Args:
#             num_windows: Number of windows to load

#         Returns:
#             List of dictionaries containing window data
#         """
#         windows = []

#         # Get sorted window directories
#         window_dirs = sorted(
#             glob.glob(os.path.join(self.data_dir, 'window_*_time_*')),
#             key=lambda x: int(re.search(r'window_(\d+)', x).group(1))
#         )
#         window_dirs = window_dirs[:num_windows]

#         for window_dir in window_dirs:
#             try:
#                 # Extract window index
#                 idx_match = re.search(r'window_(\d+)', window_dir)
#                 window_idx = int(idx_match.group(1))

#                 # Load trajectory data
#                 window_data = {
#                     'index': window_idx,
#                     'initial_traj': pd.read_csv(os.path.join(window_dir,
#                                                             f'initial_trajectory_{window_idx}.csv')),
#                     'optimized_traj': pd.read_csv(os.path.join(window_dir,
#                                                               f'optimized_trajectory_{window_idx}.csv'))
#                 }

#                 # Load boundaries
#                 boundaries = np.load(os.path.join(window_dir,
#                                                   f'boundaries_{window_idx}.npz'))
#                 window_data.update({
#                     'left_boundary': boundaries['left_boundary'],
#                     'right_boundary': boundaries['right_boundary']
#                 })

#                 windows.append(window_data)

#             except Exception as e:
#                 print(f"Error loading window {window_idx}: {str(e)}")
#                 continue

#         print(f"Loaded {len(windows)} trajectory windows")
#         return windows

#     def _find_join_point(self) -> Tuple[Optional[int], Optional[float]]:
#         """
#         Find join point between current and next trajectory.

#         Returns:
#             Tuple of (join_index, join_time) or (None, None) if not found
#         """
#         if not self.next_window_data:
#             return None, None

#         # Get first point of next trajectory
#         next_traj = self.next_window_data['optimized_traj']
#         join_point = next_traj.iloc[0]

#         # Find matching point in current trajectory
#         curr_traj = self.current_window_data['optimized_traj']
#         join_idx = None
#         min_dist = float('inf')

#         # Search last 25% of current trajectory
#         start_idx = int(len(curr_traj) * 0.75)
#         for i in range(start_idx, len(curr_traj)):
#             curr_point = curr_traj.iloc[i]
#             dist = np.sqrt((curr_point.x - join_point.x) ** 2 +
#                            (curr_point.y - join_point.y) ** 2)
#             if dist < min_dist:
#                 min_dist = dist
#                 join_idx = i

#         # Get the time of the join point in the current trajectory
#         join_time_curr = curr_traj.iloc[join_idx]['time'] if join_idx is not None else None

#         # Get the time of the join point in the next trajectory (time at index 0)
#         join_time_next = next_traj.iloc[0]['time'] if next_traj is not None else None

#         return join_idx, join_time_curr, join_time_next

#     def _update_robot_marker(self, x: float, y: float, heading: float, velocity: float):
#         """
#         Update robot marker position and orientation.

#         Args:
#             x, y: Position coordinates
#             heading: Robot heading in radians
#             velocity: Current velocity
#         """
#         # Center robot on trajectory point
#         center_x = x - (self.robot_size[0] / 2 * np.cos(heading) -
#                        self.robot_size[1] / 2 * np.sin(heading))
#         center_y = y - (self.robot_size[0] / 2 * np.sin(heading) +
#                        self.robot_size[1] / 2 * np.cos(heading))

#         # Update robot position and orientation
#         self.robot.set_xy((center_x, center_y))
#         self.robot.angle = np.degrees(heading)

#         # Update velocity text position
#         if hasattr(self, 'velocity_text'):
#             self.velocity_text.remove()
#         self.velocity_text = self.ax.text(
#             x, y + 0.5,
#             f'{velocity:.2f} m/s',
#             ha='center',
#             va='bottom',
#             color='#2E4053'  # A nice dark blue
#         )

#     def init_animation(self):
#         """Initialize animation state."""
#         # Plot boundaries
#         for window in self.windows:
#             self.ax.plot(window['left_boundary'][:, 0],
#                          window['left_boundary'][:, 1],
#                          'k-', linewidth=3, alpha=0.8)
#             self.ax.plot(window['right_boundary'][:, 0],
#                          window['right_boundary'][:, 1],
#                          'k-', linewidth=3, alpha=0.8)

#         # Initialize windows
#         self.current_window_data = self.windows[0]
#         if len(self.windows) > 1:
#             self.next_window_data = self.windows[1]

#         # Calculate axis limits
#         self._set_axis_limits()

#         # Plot start and goal
#         self._plot_start_goal_points()

#         self.ax.grid(True)

#         # Plot initial trajectory (only once for each window)
#         for i, window in enumerate(self.windows):
#             initial_traj = window['initial_traj']
#             # line, = self.ax.plot(initial_traj.x, initial_traj.y, ':', color='#707B7C', linewidth=2)
#             line, = self.ax.plot(initial_traj.x, initial_traj.y, ':', color='#FFFFFF', linewidth=2)
#             self.initial_traj_lines[i] = line  # Store the line object

#             # Store optimized trajectories for each window
#             optimized_traj = window['optimized_traj']
#             self.optimized_trajectories[i] = optimized_traj

#             if i == 0:
#                 line.set_label('Initial Trajectory')

#         self.ax.legend()
#         return self.robot, self.time_text, self.velocity_text, self.window_text

#     def _set_axis_limits(self):
#         """Calculate and set axis limits from all trajectories."""
#         x_coords = []
#         y_coords = []
#         for window in self.windows:
#             traj = window['optimized_traj']
#             x_coords.extend([traj.x.min(), traj.x.max()])
#             y_coords.extend([traj.y.min(), traj.y.max()])

#             # Add boundary points
#             x_coords.extend([
#                 window['left_boundary'][:, 0].min(),
#                 window['left_boundary'][:, 0].max(),
#                 window['right_boundary'][:, 0].min(),
#                 window['right_boundary'][:, 0].max()
#             ])
#             y_coords.extend([
#                 window['left_boundary'][:, 1].min(),
#                 window['left_boundary'][:, 1].max(),
#                 window['right_boundary'][:, 1].min(),
#                 window['right_boundary'][:, 1].max()
#             ])

#         margin = 2.0
#         self.ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
#         self.ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)

#     def _plot_start_goal_points(self):
#         """Plot start and goal points."""
#         start = self.windows[0]['optimized_traj'].iloc[0]
#         goal = self.windows[-1]['optimized_traj'].iloc[-1]

#         self.ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
#         self.ax.plot(goal.x, goal.y, 'ro', markersize=10, label='Goal')

#     def update_frame(self, frame: int):
#         """
#         Update animation frame based on trajectory time.
#         """
#         self.current_frame_time += self.time_step
#         adjusted_time = self.current_frame_time
#         current_traj = self.current_window_data['optimized_traj']
#         window_index = self.windows.index(self.current_window_data)

#         # Check if we need to switch windows
#         if adjusted_time > current_traj['time'].iloc[-1] + self.time_offset:
#             if self.next_window_data:
#                 self.time_offset += current_traj['time'].iloc[-1]
#                 adjusted_time = self.current_frame_time
#                 self.current_window_data = self.next_window_data
#                 current_traj = self.current_window_data['optimized_traj']

#                 # Update next_window_data here
#                 if window_index + 1 < len(self.windows):
#                     self.next_window_data = self.windows[window_index + 1]
#                 else:
#                     self.next_window_data = None

#                 print(f"Switched to window {self.current_window_data['index']} at adjusted_time: {adjusted_time:.2f}, offset: {self.time_offset:.2f}")
#                 self.plotted_next_window = False

#             else:
#                 # No more windows, stop updating
#                 return self.robot, self.time_text, self.velocity_text, self.window_text

#         # Find the closest point based on adjusted time
#         time_diffs = np.abs(current_traj['time'] - (adjusted_time - self.time_offset))
#         point_idx = np.argmin(time_diffs)
#         point = current_traj.iloc[point_idx]
        
#         join_idx = None
#         # Handle Trajectory Blending
#         if self.next_window_data:
#             join_idx, join_time_curr, join_time_next = self._find_join_point()
#             if join_idx is not None:
#                 if adjusted_time >= join_time_curr - self.time_step:
#                     if not self.plotted_next_window:
#                         next_traj = self.next_window_data['optimized_traj']
#                         if self.next_traj_line:
#                             self.next_traj_line.remove()

#                         self.next_traj_line, = self.ax.plot(next_traj.x, next_traj.y, 'b-', alpha=0.3)
#                         self.plotted_next_window = True

#                         if self.join_point_marker:
#                             self.join_point_marker.remove()
#                         join_point = current_traj.iloc[join_idx]
#                         self.join_point_marker = self.ax.plot(join_point.x, join_point.y, 'yo', markersize=8, alpha=0.8)[0]

#                 if adjusted_time >= self.time_offset + join_time_next:
#                     # Get the index of the current window
#                     window_index = self.windows.index(self.current_window_data)

#                     # Switch to the next trajectory
#                     self.current_window_data = self.next_window_data
#                     current_traj = self.current_window_data['optimized_traj']

#                     # Update next_window_data for the next iteration
#                     if window_index + 1 < len(self.windows):
#                         self.next_window_data = self.windows[window_index + 1]
#                     else:
#                         self.next_window_data = None

#                     self.time_offset = self.current_frame_time - join_time_next
#                     adjusted_time = join_time_next
#                     self.plotted_next_window = False
#                     print(f"Switched to window {self.current_window_data['index']} at adjusted_time: {adjusted_time:.2f}, offset: {self.time_offset:.2f}")

#                     if self.next_traj_line:
#                         self.next_traj_line.remove()
#                         self.next_traj_line = None

#                     time_diffs = np.abs(current_traj['time'] - (adjusted_time))
#                     point_idx = np.argmin(time_diffs)
#                     point = current_traj.iloc[point_idx]

#         self._update_robot_marker(point.x, point.y, point.heading, point.velocity)
#         self._update_text_displays(adjusted_time)
#         self._update_trajectory_visualization(point_idx, point, join_idx, window_index)

#         return self.robot, self.time_text, self.velocity_text, self.window_text

#     def _update_text_displays(self, adjusted_time):
#         """Update text displays with current state."""
#         self.time_text.set_text(f'Time: {adjusted_time:.2f}s')
#         self.window_text.set_text(
#             f'Window: {self.current_window_data["index"]}'
#         )

#     def _update_trajectory_visualization(self, point_idx: int, point, join_idx, window_index):
#         current_traj = self.current_window_data['optimized_traj']
#         initial_traj = self.current_window_data['initial_traj']

#         # Plot optimized trajectory (solid blue)
#         if self.current_traj_line:
#             self.current_traj_line.remove()

#         # if current window is 0 and point_idx is greater or equal to join_idx, plot optimized traj till join_idx
#         if join_idx is not None and point_idx >= join_idx and window_index < len(self.windows) - 1:
#             self.current_traj_line, = self.ax.plot(current_traj.x[:join_idx + 1], current_traj.y[:join_idx + 1],
#                                                     'b-', linewidth=2,
#                                                     label='Optimized Trajectory' if window_index == 0 else "")
#         else:
#             self.current_traj_line, = self.ax.plot(current_traj.x, current_traj.y, 'b-', linewidth=2,
#                                                     label='Optimized Trajectory' if window_index == 0 else "")

#         # Update traversed path
#         if window_index > 0:
#             # For subsequent windows, concatenate with the previous windows' trajectories
#             prev_trajectories = pd.concat([self.optimized_trajectories[i] for i in range(window_index)])
#             traversed_path_data = pd.concat([prev_trajectories, current_traj.iloc[:point_idx + 1]])
#         else:
#             # For the first window, just use the current trajectory up to the current point
#             traversed_path_data = current_traj.iloc[:point_idx + 1]

#         if self.traversed_path_line:
#             self.traversed_path_line.remove()

#         label_added = False
#         for line in self.ax.lines:
#             if line.get_label() == "Traversed Path":
#                 label_added = True
#                 break

#         # if not label_added:
#         #     self.traversed_path_line, = self.ax.plot(
#         #         traversed_path_data.x, traversed_path_data.y,
#         #         color='#707B7C', linewidth=2, label="Traversed Path"
#         #     )
#         # else:
#         #     self.traversed_path_line, = self.ax.plot(
#         #         traversed_path_data.x, traversed_path_data.y,
#         #         color='#707B7C', linewidth=2
#         #     )
#         if not label_added:
#             self.traversed_path_line, = self.ax.plot(
#                 traversed_path_data.x, traversed_path_data.y,
#                 color='#FFFFFF', linewidth=2, label="Traversed Path"
#             )
#         else:
#             self.traversed_path_line, = self.ax.plot(
#                 traversed_path_data.x, traversed_path_data.y,
#                 color='#FFFFFF', linewidth=2
#             )

#         # Clear initial and optimized trajectories up to the current point
#         current_initial_traj_line = self.initial_traj_lines[window_index]
#         current_initial_traj_line.set_data(initial_traj.x[point_idx:], initial_traj.y[point_idx:])

#         if join_idx is not None and point_idx >= join_idx and window_index < len(self.windows) - 1:
#             self.current_traj_line.set_data(current_traj.x[point_idx:], current_traj.y[point_idx:])
#         else:
#             self.current_traj_line.set_data(current_traj.x[point_idx:], current_traj.y[point_idx:])

#         # Remove duplicate labels from the legend
#         handles, labels = self.ax.get_legend_handles_labels()
#         by_label = dict(zip(labels, handles))
#         self.ax.legend(by_label.values(), by_label.keys())

#     def create_animation(self, output_file: str = 'trajectory_animation.mp4'):
#         """
#         Create and save animation.

#         Args:
#             output_file: Output video file path
#         """
#         # Calculate total animation time based on last window's last time entry
#         total_time = self.windows[-1]['optimized_traj']['time'].iloc[-1] + self.time_offset
#         num_frames = int(total_time / self.time_step)

#         print(f"Creating animation with {num_frames} frames "
#               f"for {total_time:.2f} seconds...")

#         # Create animation
#         anim = animation.FuncAnimation(
#             self.fig,
#             self.update_frame,
#             init_func=self.init_animation,
#             frames=num_frames,
#             interval=self.frame_interval,
#             blit=True
#         )

#         # Save animation
#         print(f"Saving animation to {output_file}...")
#         writer = animation.FFMpegWriter(fps=self.fps)
#         anim.save(output_file, writer=writer)
#         print("Animation saved successfully!")

# def main():
#     """Main entry point."""
#     num_windows = 30 # Example: Process all windows
#     animator = TrajectoryAnimator(num_windows=num_windows)
#     animator.create_animation()

# if __name__ == "__main__":
#     main()



        

# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.patches import Rectangle
# import os
# import pandas as pd
# import re
# import glob
# from typing import List, Dict, Tuple, Optional

# class TrajectoryAnimator:
#     """
#     Class for creating animations of trajectory data from multiple windows
#     with clear visualization of trajectory optimization and robot movement.
#     """

#     # Define distinct colors for different trajectory types
#     COLORS = {
#         'initial': '#FFA500',  # Orange for initial trajectory
#         'optimized': '#00FF00',  # Green for optimized trajectory
#         'traversed': '#FF00FF',  # Magenta for traversed path
#         'robot': '#FF5733',  # Bright orange for robot
#         'boundaries': '#000000',  # Black for boundaries
#         'join_point': '#FFFF00'  # Yellow for join points
#     }

#     def __init__(self, base_dir: str = None, num_windows: int = None):
#         """Initialize animator with configuration."""
#         # Set up data directory
#         self.data_dir = "/home/abhi/ros2_lanelet2_ws/trajectory_data/data" if base_dir is None else base_dir
#         print(f"Using data directory: {self.data_dir}")

#         # Animation settings
#         self.fps = 30
#         self.frame_interval = 1000 / self.fps
#         self.robot_size = (0.5, 0.3)  # width, height in meters

#         # Load windows
#         if num_windows is None:
#             num_windows = len(glob.glob(os.path.join(self.data_dir, 'window_*_time_*')))
#         self.windows = self._load_windows(num_windows)

#         # Initialize plot
#         self.fig, self.ax = plt.subplots(figsize=(12, 8))
#         self.ax.set_aspect('equal')

#         # Initialize display elements
#         self._init_text_displays()

#         # Initialize robot marker
#         self.robot = Rectangle((0, 0), self.robot_size[0], self.robot_size[1],
#                                fc=self.COLORS['robot'], alpha=0.7)
#         self.ax.add_patch(self.robot)

#         # State tracking
#         self.current_window_data = None
#         self.next_window_data = None
#         self.time_offset = 0
#         self.current_frame_time = 0
#         self.time_step = 0.02

#         # Visualization elements
#         self.initial_traj_line = None  # Current window's initial trajectory
#         self.optimized_traj_line = None  # Current window's optimized trajectory
#         self.traversed_path_line = None  # Continuous traversed path
#         self.join_point_marker = None  # Marker for trajectory join points
#         self.next_window_preview = None  # Preview of next window's trajectory
#         self.next_traj_line = None # line of next window traj

#         # Store full path data
#         self.traversed_points = []  # Store all traversed points for continuous path
#         self.optimized_trajectories = {}  # Store all optimized trajectories

#          # initialize a flag to plot next window
#         self.plotted_next_window = False

#     def _init_text_displays(self):
#         """Initialize text display elements on plot."""
#         self.time_text = self.ax.text(0.4, 1.02, '',
#                                      transform=self.ax.transAxes,
#                                      ha='center')
#         self.window_text = self.ax.text(0.02, 1.02, '',
#                                        transform=self.ax.transAxes)
#         self.velocity_text = self.ax.text(0, 0, '', visible=False)

#     def _load_windows(self, num_windows: int) -> List[Dict]:
#         """Load specified number of trajectory windows."""
#         windows = []
#         self.optimized_trajectories = {}  # Initialize the dictionary here
#         window_dirs = sorted(
#             glob.glob(os.path.join(self.data_dir, 'window_*_time_*')),
#             key=lambda x: int(re.search(r'window_(\d+)', x).group(1))
#         )
#         window_dirs = window_dirs[:num_windows]

#         for window_dir in window_dirs:
#             try:
#                 idx_match = re.search(r'window_(\d+)', window_dir)
#                 window_idx = int(idx_match.group(1))

#                 window_data = {
#                     'index': window_idx,
#                     'initial_traj': pd.read_csv(os.path.join(window_dir,
#                                                             f'initial_trajectory_{window_idx}.csv')),
#                     'optimized_traj': pd.read_csv(os.path.join(window_dir,
#                                                               f'optimized_trajectory_{window_idx}.csv'))
#                 }

#                 boundaries = np.load(os.path.join(window_dir,
#                                                   f'boundaries_{window_idx}.npz'))
#                 window_data.update({
#                     'left_boundary': boundaries['left_boundary'],
#                     'right_boundary': boundaries['right_boundary']
#                 })

#                 self.optimized_trajectories[window_idx] = window_data['optimized_traj']
#                 windows.append(window_data)

#             except Exception as e:
#                 print(f"Error loading window {window_idx}: {str(e)}")
#                 continue

#         print(f"Loaded {len(windows)} trajectory windows")
#         return windows

#     def _set_axis_limits(self):
#         """Calculate and set axis limits from all trajectories."""
#         x_coords = []
#         y_coords = []
#         for window in self.windows:
#             traj = window['optimized_traj']
#             x_coords.extend([traj.x.min(), traj.x.max()])
#             y_coords.extend([traj.y.min(), traj.y.max()])

#             # Add boundary points
#             x_coords.extend([
#                 window['left_boundary'][:, 0].min(),
#                 window['left_boundary'][:, 0].max(),
#                 window['right_boundary'][:, 0].min(),
#                 window['right_boundary'][:, 0].max()
#             ])
#             y_coords.extend([
#                 window['left_boundary'][:, 1].min(),
#                 window['left_boundary'][:, 1].max(),
#                 window['right_boundary'][:, 1].min(),
#                 window['right_boundary'][:, 1].max()
#             ])

#         margin = 2.0
#         self.ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
#         self.ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)

#     def _plot_global_boundaries(self):
#         """Plot boundaries for complete global path."""
#         for window in self.windows:
#             self.ax.plot(window['left_boundary'][:, 0],
#                          window['left_boundary'][:, 1],
#                          '-', color=self.COLORS['boundaries'],
#                          linewidth=2, alpha=0.8)
#             self.ax.plot(window['right_boundary'][:, 0],
#                          window['right_boundary'][:, 1],
#                          '-', color=self.COLORS['boundaries'],
#                          linewidth=2, alpha=0.8)

#     def _plot_start_goal_points(self):
#         """Plot start and goal points."""
#         start = self.windows[0]['optimized_traj'].iloc[0]
#         goal = self.windows[-1]['optimized_traj'].iloc[-1]

#         self.ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
#         self.ax.plot(goal.x, goal.y, 'ro', markersize=10, label='Goal')

#     def init_animation(self):
#         """Initialize animation state and plot global path."""
#         # First plot global path boundaries
#         self._plot_global_boundaries()

#         # Plot start and goal points
#         self._plot_start_goal_points()

#         # Initialize first window
#         self.current_window_data = self.windows[0]
#         if len(self.windows) > 1:
#             self.next_window_data = self.windows[1]

#         # Calculate and set axis limits
#         self._set_axis_limits()

#         # Set up grid and initial legend
#         self.ax.grid(True)
#         self.ax.legend()

#         return self.robot, self.time_text, self.velocity_text, self.window_text

#     def _find_join_point(self) -> Tuple[Optional[int], Optional[float], Optional[float]]:
#         """
#         Find join point between current and next trajectory.
#         The join point is defined as the point in the current trajectory that has
#         the smallest time difference to the first point of the next trajectory
#         Returns:
#             Tuple of (join_index, join_time) or (None, None) if not found
#         """
#         if not self.next_window_data:
#             return None, None, None

#         # Get first point of next trajectory
#         next_traj = self.next_window_data['optimized_traj']
#         join_point_next = next_traj.iloc[0]
#         join_time_next = join_point_next['time']

#         # Find matching point in current trajectory
#         curr_traj = self.current_window_data['optimized_traj']
#         join_idx = None
#         min_time_diff = float('inf')

#         # Search from the 75% of the current trajectory
#         start_idx = int(len(curr_traj) * 0.75)
#         for i in range(start_idx, len(curr_traj)):
#             curr_point = curr_traj.iloc[i]
#             time_diff = abs(curr_point['time'] - join_time_next)
#             if time_diff < min_time_diff:
#                 min_time_diff = time_diff
#                 join_idx = i

#         # Get the time of the join point in the current trajectory
#         join_time_curr = curr_traj.iloc[join_idx]['time'] if join_idx is not None else None

#         return join_idx, join_time_curr, join_time_next
    
#     def _switch_to_next_window(self):
#         """Handle transition to next window."""

#         window_index = self.windows.index(self.current_window_data)

#         # update time_offset using join_time_curr
#         _, join_time_curr, _ = self._find_join_point()

#         self.time_offset += join_time_curr
#         self.current_frame_time = self.time_offset

#         self.current_window_data = self.next_window_data

#         # Update next window data
#         if window_index + 1 < len(self.windows):
#             self.next_window_data = self.windows[window_index + 1]
#         else:
#             self.next_window_data = None

#         # Clear current window visualization
#         if self.initial_traj_line:
#             self.initial_traj_line.remove()
#             self.initial_traj_line = None
#         if self.optimized_traj_line:
#             self.optimized_traj_line.remove()
#             self.optimized_traj_line = None
#         if self.next_window_preview:
#             self.next_window_preview.remove()
#             self.next_window_preview = None
#         if self.join_point_marker:
#             self.join_point_marker.remove()
#             self.join_point_marker = None
        
#         self.plotted_next_window = False

#         print(f"Switched to window {self.current_window_data['index']} at time: {self.current_frame_time:.2f}s")

#     def _show_next_window_preview(self):
#         """Show preview of next window's trajectory."""
#         if not self.next_window_preview and self.next_window_data:
#             next_traj = self.next_window_data['optimized_traj']
#             self.next_window_preview, = self.ax.plot(
#                 next_traj.x, next_traj.y,
#                 '--', color=self.COLORS['optimized'],
#                 alpha=0.3, linewidth=2
#             )

#     def _add_to_traversed_path(self, point_pos: np.ndarray):
#         """Add point to traversed path history."""
#         self.traversed_points.append(point_pos)

#         # Update traversed path visualization
#         if len(self.traversed_points) > 1:
#             points = np.array(self.traversed_points)
#             if self.traversed_path_line:
#                 self.traversed_path_line.remove()
#             self.traversed_path_line, = self.ax.plot(
#                 points[:, 0], points[:, 1],
#                 '-', color=self.COLORS['traversed'],
#                 linewidth=3,
#                 label='Traversed Path' if len(points) == 2 else ""
#             )

#     def _update_robot_marker(self, x: float, y: float, heading: float, velocity: float):
#         """Update robot marker position and orientation."""
#         # Add current position to traversed path
#         self._add_to_traversed_path(np.array([x, y]))

#         # Center robot on trajectory point
#         center_x = x - (self.robot_size[0] / 2 * np.cos(heading) -
#                         self.robot_size[1] / 2 * np.sin(heading))
#         center_y = y - (self.robot_size[0] / 2 * np.sin(heading) +
#                         self.robot_size[1] / 2 * np.cos(heading))

#         # Update robot position and orientation
#         self.robot.set_xy((center_x, center_y))
#         self.robot.angle = np.degrees(heading)

#         # Update velocity text display
#         if hasattr(self, 'velocity_text'):
#             self.velocity_text.remove()
#         self.velocity_text = self.ax.text(
#             x, y + 0.5,
#             f'{velocity:.2f} m/s',
#             ha='center',
#             va='bottom',
#             color='#2E4053'
#         )

#     def _update_text_displays(self, adjusted_time):
#         """Update text displays with current state."""
#         self.time_text.set_text(f'Time: {adjusted_time:.2f}s')
#         self.window_text.set_text(
#             f'Window: {self.current_window_data["index"]}'
#         )

#     def _update_trajectory_visualization(self, point_idx: int, point, join_idx: Optional[int], window_index: int):
#         """Update trajectory visualization for current window."""
#         current_traj = self.current_window_data['optimized_traj']
#         initial_traj = self.current_window_data['initial_traj']

#         # Clear previous trajectory lines
#         if self.initial_traj_line:
#             self.initial_traj_line.remove()
#         if self.optimized_traj_line:
#             self.optimized_traj_line.remove()

#         # Plot initial trajectory for current window
#         self.initial_traj_line, = self.ax.plot(
#             initial_traj.x, initial_traj.y,
#             '--', color=self.COLORS['initial'],
#             linewidth=2, label='Initial Trajectory' if window_index == 0 else None
#         )

#         # Plot optimized trajectory up to join point or end
#         if join_idx is not None and point_idx >= join_idx and window_index < len(self.windows) - 1:
#             # Show optimized path only up to join point
#             self.optimized_traj_line, = self.ax.plot(
#                 current_traj.x[:join_idx + 1],
#                 current_traj.y[:join_idx + 1],
#                 '-', color=self.COLORS['optimized'],
#                 linewidth=2, label='Optimized Trajectory' if window_index == 0 else None
#             )
#         else:
#             # Show remaining optimized path
#             self.optimized_traj_line, = self.ax.plot(
#                 current_traj.x, current_traj.y,
#                 '-', color=self.COLORS['optimized'],
#                 linewidth=2, label='Optimized Trajectory' if window_index == 0 else None
#             )

#         # Remove duplicate labels from the legend
#         handles, labels = self.ax.get_legend_handles_labels()
#         by_label = dict(zip(labels, handles))
#         self.ax.legend(by_label.values(), by_label.keys())
    
#     def update_frame(self, frame: int):
#         """Update animation frame."""
#         self.current_frame_time += self.time_step
#         adjusted_time = self.current_frame_time
#         current_traj = self.current_window_data['optimized_traj']
#         window_index = self.windows.index(self.current_window_data)

#         # Handle trajectory blending
#         join_idx = None
#         if self.next_window_data:
#             join_idx, join_time_curr, join_time_next = self._find_join_point()
#             if join_idx is not None:
#                 time_to_join = join_time_curr - (adjusted_time - self.time_offset)
                
#                 if adjusted_time >= self.time_offset + join_time_curr - self.time_step:
#                     if not self.plotted_next_window:
#                         next_traj = self.next_window_data['optimized_traj']
#                         if self.next_traj_line:
#                             self.next_traj_line.remove()

#                         self.next_traj_line, = self.ax.plot(next_traj.x, next_traj.y, 'b-', alpha=0.3)
#                         self.plotted_next_window = True

#                         if self.join_point_marker:
#                             self.join_point_marker.remove()

#                         # Plot the join point
#                         join_point = current_traj.iloc[join_idx]
#                         self.join_point_marker = self.ax.plot(
#                             join_point.x, join_point.y,
#                             'o', color=self.COLORS['join_point'], markersize=8, alpha=0.8
#                         )[0]

#         # Check if it's time to switch to the next window
#         if join_idx is not None and adjusted_time >= self.time_offset + current_traj['time'].iloc[join_idx]:
#             self._switch_to_next_window()
#             current_traj = self.current_window_data['optimized_traj']
#             window_index = self.windows.index(self.current_window_data)

#             # Update visualizations for the new window
#             self._update_trajectory_visualization(0,  # Start from the beginning of the new trajectory
#                                                     self.current_window_data['optimized_traj'].iloc[0],
#                                                     None, window_index)
#             self._update_robot_marker(self.current_window_data['optimized_traj'].iloc[0].x,
#                                       self.current_window_data['optimized_traj'].iloc[0].y,
#                                       self.current_window_data['optimized_traj'].iloc[0].heading,
#                                       self.current_window_data['optimized_traj'].iloc[0].velocity)
#             self._update_text_displays(self.time_offset)  # Use time_offset for initial display in the new window

#             return self.robot, self.time_text, self.velocity_text, self.window_text

#         # Find the closest point based on adjusted time
#         time_diffs = np.abs(current_traj['time'] - (adjusted_time - self.time_offset))
#         point_idx = np.argmin(time_diffs)
#         point = current_traj.iloc[point_idx]

#         # Update visualizations
#         self._update_robot_marker(point.x, point.y, point.heading, point.velocity)
#         self._update_text_displays(adjusted_time)
#         self._update_trajectory_visualization(point_idx, point, join_idx, window_index)

#         return self.robot, self.time_text, self.velocity_text, self.window_text

#     def create_animation(self, output_file: str = 'trajectory_animation.mp4'):
#         """Create and save animation."""
#         total_time = self.windows[-1]['optimized_traj']['time'].iloc[-1] + self.time_offset
#         num_frames = int(total_time / self.time_step)

#         print(f"Creating animation with {num_frames} frames "
#               f"for {total_time:.2f} seconds...")

#         anim = animation.FuncAnimation(
#             self.fig,
#             self.update_frame,
#             init_func=self.init_animation,
#             frames=num_frames,
#             interval=self.frame_interval,
#             blit=True
#         )

#         print(f"Saving animation to {output_file}...")
#         writer = animation.FFMpegWriter(fps=self.fps)
#         anim.save(output_file, writer=writer)
#         print("Animation saved successfully!")

# def main():
#     """Main function to run the trajectory animation."""
#     # Get number of windows
#     data_dir = "/home/abhi/ros2_lanelet2_ws/trajectory_data/data"  # Make sure this is correct
#     num_windows = len(glob.glob(os.path.join(data_dir, 'window_*_time_*')))

#     # Create animator object
#     animator = TrajectoryAnimator(num_windows=num_windows)

#     # Create and save the animation
#     animator.create_animation('my_animation.mp4')

# if __name__ == "__main__":
#     main()





from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import os
import pandas as pd
import re
import glob
from typing import List, Dict, Tuple, Optional
import math
import time

class TrajectoryAnimator:
    def __init__(self, base_dir: str = None, num_windows: int = None):
        """
        Initialize animator with configuration.

        Args:
            base_dir: Base directory for trajectory data
            num_windows: Number of windows to load for animation
        """
        # Set up data directory
        self.data_dir = "/home/abhi/ros2_lanelet2_ws/trajectory_data/data" if base_dir is None else base_dir
        print(f"Using data directory: {self.data_dir}")

        # Animation settings
        self.fps = 35
        self.frame_interval = 1000 / self.fps  # milliseconds
        self.robot_size = (1.3, 0.6)  # width, height in meters

        # Load trajectory windows
        if num_windows is None:
            # Get the number of window directories
            num_windows = len(glob.glob(os.path.join(self.data_dir, 'window_*_time_*')))
        self.windows = self._load_windows(num_windows)
        
        # Store preprocessed boundaries
        self.left_boundary, self.right_boundary = self._preprocess_boundaries()

        # Initialize plot
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_aspect('equal')

        # Initialize text displays
        self._init_text_displays()

        # Initialize robot marker
        self.robot = Rectangle((0, 0), self.robot_size[0], self.robot_size[1],
                               fc='#FF8000',  # Use a nice orange color
                               alpha=1.0)
        self.ax.add_patch(self.robot)

        # Window tracking
        self.current_window_data = None
        self.next_window_data = None
        self.time_offset = 0  # Initialize time offset 
        self.current_frame_time = 0  # Initialize the time for the current frame
        self.time_step = 0.02  # Time increment for each frame (adjust as needed)

        # initialize a flag to plot next window
        self.plotted_next_window = False
        self.current_traj_line = None
        self.next_traj_line = None

        # Traversed path
        self.traversed_path_line = None

        # Join point marker
        self.join_point_marker = None

        # Initial trajectory lines for each window
        self.initial_traj_line = None

        # Store optimized trajectories for all windows
        self.optimized_traj_line = None

        # Precompute join points and total number of points
        self.join_points = self._calculate_join_points()
        self.total_points = self._calculate_total_points()
        self.num_frames = self.total_points  # Use total points as frames

        # Traversed path
        self.traversed_path_data = pd.DataFrame(columns=['x', 'y'])  # Initialize empty DataFrame
        self.traversed_path_line = None  # This will be the line object for the traversed path

        # Initialize frame index
        self.frame_idx = 0

    def _init_text_displays(self):
        """Initialize text display elements on plot."""
        self.time_text = self.ax.text(0.4, 1.02, '',
                                      transform=self.ax.transAxes,
                                      ha='center')
        self.window_text = self.ax.text(0.02, 1.02, '',
                                        transform=self.ax.transAxes)
        self.velocity_text = self.ax.text(0, 0, '', visible=False)

    def _load_windows(self, num_windows: int) -> List[Dict]:
        """
        Load specified number of trajectory windows.

        Args:
            num_windows: Number of windows to load

        Returns:
            List of dictionaries containing window data
        """
        windows = []

        # Get sorted window directories
        window_dirs = sorted(
            glob.glob(os.path.join(self.data_dir, 'window_*_time_*')),
            key=lambda x: int(re.search(r'window_(\d+)', x).group(1))
        )
        window_dirs = window_dirs[:num_windows]

        for window_dir in window_dirs:
            try:
                # Extract window index
                idx_match = re.search(r'window_(\d+)', window_dir)
                window_idx = int(idx_match.group(1))

                # Load trajectory data
                window_data = {
                    'index': window_idx,
                    'initial_traj': pd.read_csv(os.path.join(window_dir,
                                                            f'initial_trajectory_{window_idx}.csv')),
                    'optimized_traj': pd.read_csv(os.path.join(window_dir,
                                                              f'optimized_trajectory_{window_idx}.csv'))
                }

                # Load boundaries
                boundaries = np.load(os.path.join(window_dir,
                                                  f'boundaries_{window_idx}.npz'))
                window_data.update({
                    'left_boundary': boundaries['left_boundary'],
                    'right_boundary': boundaries['right_boundary']
                })

                windows.append(window_data)

            except Exception as e:
                print(f"Error loading window {window_idx}: {str(e)}")
                continue

        print(f"Loaded {len(windows)} trajectory windows")
        return windows

    def _calculate_join_points(self) -> List[int]:
        """Calculates join points between consecutive windows."""
        join_points = []
        for i in range(len(self.windows) - 1):
            current_window = self.windows[i]
            next_window = self.windows[i + 1]

            # Get the first point of the next trajectory
            next_traj_first_point = next_window['optimized_traj'].iloc[0]

            # Iterate over the current trajectory to find the matching point
            join_idx = None
            for j in range(len(current_window['optimized_traj'])):
                current_point = current_window['optimized_traj'].iloc[j]
                # Compare x, y (and optionally other attributes) for a match
                if np.allclose(current_point.x, next_traj_first_point.x) and np.allclose(current_point.y, next_traj_first_point.y):
                    join_idx = j
                    break

            if join_idx is not None:
                join_points.append(join_idx)
            else:
                print(f"Warning: No join point found between window {i} and {i+1}")
                join_points.append(len(current_window['optimized_traj'])-1) # Fallback: use the last point as join point

        return join_points

    def _calculate_total_points(self) -> int:
        """Calculates the total number of unique points to be traversed."""
        total_points = 0
        for i, window in enumerate(self.windows):
            if i < len(self.windows) - 1:
                # Add points up to the join point index
                total_points += self.join_points[i] + 1
            else:
                # Last window, add all points
                total_points += len(window['optimized_traj'])
        return total_points

    def _update_robot_marker(self, x: float, y: float, heading: float, velocity: float):
        """
        Update robot marker position and orientation.

        Args:
            x, y: Position coordinates
            heading: Robot heading in radians
            velocity: Current velocity
        """
        # Center robot on trajectory point
        center_x = x - (self.robot_size[0] / 2 * np.cos(heading) -
                       self.robot_size[1] / 2 * np.sin(heading))
        center_y = y - (self.robot_size[0] / 2 * np.sin(heading) +
                       self.robot_size[1] / 2 * np.cos(heading))

        # Update robot position and orientation
        self.robot.set_xy((center_x, center_y))
        self.robot.angle = np.degrees(heading)

        # Update velocity text position
        if hasattr(self, 'velocity_text'):
            self.velocity_text.remove()
        self.velocity_text = self.ax.text(
            x, y + 0.5,
            f'{velocity:.2f} m/s',
            ha='center',
            va='bottom',
            color='#000000'  # A nice dark
        )

    # def init_animation(self):
    #     """Initialize animation state."""
    #     # Plot boundaries
    #     for window in self.windows:
    #         self.ax.plot(window['left_boundary'][:, 0],
    #                      window['left_boundary'][:, 1],
    #                      'k-', linewidth=3, alpha=0.8)
    #         self.ax.plot(window['right_boundary'][:, 0],
    #                      window['right_boundary'][:, 1],
    #                      'k-', linewidth=3, alpha=0.8)

    #     # Initialize windows
    #     self.current_window_data = self.windows[0]
    #     if len(self.windows) > 1:
    #         self.next_window_data = self.windows[1]

    #     # Calculate axis limits
    #     self._set_axis_limits()

    #     # Plot start and goal
    #     self._plot_start_goal_points()

    #     self.ax.grid(True)

    #     self.ax.legend()
    #     return self.robot, self.time_text, self.velocity_text, self.window_text # Add necessary artists

    # def _set_axis_limits(self):
    #     """Calculate and set axis limits from all trajectories."""
    #     x_coords = []
    #     y_coords = []
    #     for window in self.windows:
    #         traj = window['optimized_traj']
    #         x_coords.extend([traj.x.min(), traj.x.max()])
    #         y_coords.extend([traj.y.min(), traj.y.max()])

    #         # Add boundary points
    #         x_coords.extend([
    #             window['left_boundary'][:, 0].min(),
    #             window['left_boundary'][:, 0].max(),
    #             window['right_boundary'][:, 0].min(),
    #             window['right_boundary'][:, 0].max()
    #         ])
    #         y_coords.extend([
    #             window['left_boundary'][:, 1].min(),
    #             window['left_boundary'][:, 1].max(),
    #             window['right_boundary'][:, 1].min(),
    #             window['right_boundary'][:, 1].max()
    #         ])

    #     margin = 2.0
    #     self.ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
    #     self.ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)
    
    def _set_axis_limits(self, left_boundary, right_boundary):
        """Calculate and set axis limits from trajectories and preprocessed boundaries."""
        x_coords = []
        y_coords = []
        
        # Add boundary points
        x_coords.extend([
            left_boundary[:, 0].min(),
            left_boundary[:, 0].max(),
            right_boundary[:, 0].min(),
            right_boundary[:, 0].max()
        ])
        y_coords.extend([
            left_boundary[:, 1].min(),
            left_boundary[:, 1].max(),
            right_boundary[:, 1].min(),
            right_boundary[:, 1].max()
        ])
        
        # Add trajectory points
        for window in self.windows:
            traj = window['optimized_traj']
            x_coords.extend([traj.x.min(), traj.x.max()])
            y_coords.extend([traj.y.min(), traj.y.max()])

        margin = 2.0
        self.ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
        self.ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)

    def _plot_start_goal_points(self):
        """Plot start and goal lines across the road width."""
        start = self.windows[0]['optimized_traj'].iloc[0]
        goal = self.windows[-1]['optimized_traj'].iloc[-1]

        # Calculate start line (perpendicular to the initial heading)
        start_heading = start.heading
        start_perp_heading = start_heading + np.pi / 2  # Perpendicular heading

        # Estimate road width (you might need to adjust this)
        road_width = 1.0  # Example: Assume a road width of 1.0 meters

        # Calculate start line endpoints
        start_left_x = start.x + (road_width / 2) * np.cos(start_perp_heading)
        start_left_y = start.y + (road_width / 2) * np.sin(start_perp_heading)
        start_right_x = start.x - (road_width / 2) * np.cos(start_perp_heading)
        start_right_y = start.y - (road_width / 2) * np.sin(start_perp_heading)

        # Calculate goal line (perpendicular to the final heading)
        goal_heading = goal.heading
        goal_perp_heading = goal_heading + np.pi / 2

        # Calculate goal line endpoints
        goal_left_x = goal.x + (road_width / 2) * np.cos(goal_perp_heading)
        goal_left_y = goal.y + (road_width / 2) * np.sin(goal_perp_heading)
        goal_right_x = goal.x - (road_width / 2) * np.cos(goal_perp_heading)
        goal_right_y = goal.y - (road_width / 2) * np.sin(goal_perp_heading)

        # Plot the start and goal lines
        # Plot with labels only once
        if not any(line.get_label() == 'Start Line' for line in self.ax.lines):
            self.ax.plot([start_left_x, start_right_x], [start_left_y, start_right_y], 
                        linestyle='-', color='#7FBA00', linewidth=4, label='Start Line')
            self.ax.plot([goal_left_x, goal_right_x], [goal_left_y, goal_right_y], 
                        linestyle='-', color='#F25022', linewidth=4, label='Goal Line')
        else:
            self.ax.plot([start_left_x, start_right_x], [start_left_y, start_right_y], 
                        linestyle='-', color='#7FBA00', linewidth=4)
            self.ax.plot([goal_left_x, goal_right_x], [goal_left_y, goal_right_y], 
                        linestyle='-', color='#F25022', linewidth=4)
        
        
    def _update_text_displays(self, adjusted_time):
        """Update text displays with current state."""
        self.time_text.set_text(f'Time: {adjusted_time:.2f}s')
        self.window_text.set_text(
            f'Window: {self.current_window_data["index"]}'
        )
    
    def _update_legend(self):
        """Update legend without duplicates."""
        handles, labels = self.ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        # Sort legend items in desired order
        desired_order = ['Start Line', 'Goal Line', 'Optimized Trajectory', 
                        'Initial Trajectory', 'Traversed Path']
        sorted_items = [(label, by_label[label]) for label in desired_order 
                    if label in by_label]
        if sorted_items:
            sorted_labels, sorted_handles = zip(*sorted_items)
            self.ax.legend(sorted_handles, sorted_labels)
        

    def update_frame(self, frame: int):
        """Updates the animation for a given frame."""

        # Determine current window and point index based on frame number
        window_idx = 0
        point_idx = 0
        temp_frame = frame
        for i in range(len(self.windows)):
            if i < len(self.windows) - 1:  # Not the last window
                join_idx = self.join_points[i]
                if temp_frame <= join_idx:
                    window_idx = i
                    point_idx = temp_frame
                    break
                else:
                    temp_frame -= (join_idx + 1)
            else:  # Last window
                window_idx = i
                point_idx = temp_frame
                break

        # Get current window data
        self.current_window_data = self.windows[window_idx]
        current_traj = self.current_window_data['optimized_traj']
        initial_traj = self.current_window_data['initial_traj']

        # Get current point
        point = current_traj.iloc[point_idx]

        # Update robot position and text
        self._update_robot_marker(point.x, point.y, point.heading, point.velocity)
        self._update_text_displays(point.time)

        # Update traversed path
        if self.traversed_path_data.empty:
            self.traversed_path_data = pd.DataFrame({'x': [point.x], 'y': [point.y]})
        else:
            self.traversed_path_data = pd.concat([self.traversed_path_data, pd.DataFrame({'x': [point.x], 'y': [point.y]})], ignore_index=True)

        if self.traversed_path_line:
            self.traversed_path_line.remove()

        # Plot traversed path (gray line)
        label = 'Traversed Path' if not any(line.get_label() == 'Traversed Path' 
                                      for line in self.ax.lines) else None
        self.traversed_path_line, = self.ax.plot(
            self.traversed_path_data['x'], 
            self.traversed_path_data['y'], 
            '-', color='gray', linewidth=2,
            label=label
        )

        # Update initial and optimized trajectory lines
        if 'optimized_line' not in self.current_window_data:
            # line, = self.ax.plot(current_traj.x[:point_idx+1], current_traj.y[:point_idx+1], '-', color='blue', linewidth=2, label='Optimized Trajectory')
            line, = self.ax.plot(current_traj.x, current_traj.y, '-', 
                           color='#00A4EF', linewidth=2, 
                           label='Optimized Trajectory', alpha = 1.0)
            self.current_window_data['optimized_line'] = line
            self.optimized_line = self.current_window_data['optimized_line'] # Store reference
        else:
            # self.optimized_line.set_data(current_traj.x[:point_idx+1], current_traj.y[:point_idx+1])
            self.optimized_line.set_data(current_traj.x, current_traj.y)

        if 'initial_line' not in self.current_window_data:
            line, = self.ax.plot(initial_traj.x, initial_traj.y, ':', 
                           color='#FFB900', linewidth=2, alpha=1.0,
                           label='Initial Trajectory')
            self.current_window_data['initial_line'] = line
            self.initial_line = self.current_window_data['initial_line'] # Store reference
        else:
            self.initial_line.set_data(initial_traj.x, initial_traj.y)

        # Remove initial trajectory from previous window
        if window_idx > 0 and 'initial_line' in self.windows[window_idx - 1]:
            self.windows[window_idx - 1]['initial_line'].set_data([], [])
        if window_idx > 0 and 'optimized_line' in self.windows[window_idx - 1]:
            self.windows[window_idx - 1]['optimized_line'].set_data([], [])

        # Update legend
        self.ax.legend()
        
        # Update legend
        self._update_legend()

        # Return the updated artists
        return_artists = [self.robot, self.time_text, self.velocity_text, self.window_text, self.traversed_path_line]

        if 'optimized_line' in self.current_window_data:
            return_artists.append(self.current_window_data['optimized_line'])

        if 'initial_line' in self.current_window_data:
            return_artists.append(self.current_window_data['initial_line'])

        return return_artists
    
    def _preprocess_boundaries(self):
        """
        Combine and clean up boundary points from all windows to remove duplicates.
        """
        # Initialize lists for combined boundaries
        left_points = []
        right_points = []
        
        # Small threshold for considering points as duplicates
        threshold = 1e-6
        
        # Process each window's boundaries
        for window in self.windows:
            left_bound = window['left_boundary']
            right_bound = window['right_boundary']
            
            # Process left boundary points
            for point in left_bound:
                # Check if point is already in the list (within threshold)
                is_duplicate = any(np.all(np.abs(point - existing) < threshold) 
                                for existing in left_points)
                if not is_duplicate:
                    left_points.append(point)
            
            # Process right boundary points
            for point in right_bound:
                is_duplicate = any(np.all(np.abs(point - existing) < threshold) 
                                for existing in right_points)
                if not is_duplicate:
                    right_points.append(point)
        
        # Convert to numpy arrays and sort points
        left_points = np.array(left_points)
        right_points = np.array(right_points)
        
        # Sort points based on x-coordinate (or another criterion if needed)
        # left_points = left_points[np.argsort(left_points[:, 0])]
        # right_points = right_points[np.argsort(right_points[:, 0])]
        
        return left_points, right_points

    def init_animation(self):
        """Initialize animation state."""
        # Plot boundaries
        # for window in self.windows:
        #     self.ax.plot(window['left_boundary'][:, 0],
        #                  window['left_boundary'][:, 1],
        #                  'k-', linewidth=1.5, alpha=1.0)
        #     self.ax.plot(window['right_boundary'][:, 0],
        #                  window['right_boundary'][:, 1],
        #                  'k-', linewidth=1.5, alpha=1.0)
        
        # Preprocess boundaries to remove duplicates
        left_boundary, right_boundary = self._preprocess_boundaries()
        # Plot clean boundaries once
        self.ax.plot(left_boundary[:, 0], left_boundary[:, 1],
                    'k-', linewidth=1.5, alpha=1.0, zorder=1)
        self.ax.plot(right_boundary[:, 0], right_boundary[:, 1],
                    'k-', linewidth=1.5, alpha=1.0, zorder=1)

        # Initialize windows
        self.current_window_data = self.windows[0]
        if len(self.windows) > 1:
            self.next_window_data = self.windows[1]

        # Calculate axis limits
        # self._set_axis_limits()
        # Calculate axis limits using preprocessed boundaries
        self._set_axis_limits(left_boundary, right_boundary)

        # Plot start and goal
        self._plot_start_goal_points()

        self.ax.grid(True)
        
        self.ax.legend()

        return self.robot, self.time_text, self.velocity_text, self.window_text

    def create_animation(self, output_file: str = 'trajectory_animation.mp4'):
        """
        Create and save animation.

        Args:
            output_file: Output video file path
        """
        # Create animation
        anim = animation.FuncAnimation(
            self.fig,
            self.update_frame,
            init_func=self.init_animation,
            frames=self.num_frames,
            interval=self.frame_interval,
            blit=True
        )

        # Save animation
        print(f"Saving animation to {output_file}...")
        writer = animation.FFMpegWriter(fps=self.fps)
        anim.save(output_file, writer=writer)
        print("Animation saved successfully!")

def main():
    """Main entry point."""
    num_windows = 30 # Example: Process all windows
    animator = TrajectoryAnimator()
    animator.create_animation()

if __name__ == "__main__":
    main()