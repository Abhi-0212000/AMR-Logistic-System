"""Example demonstrating integrated use of spline and velocity profile generation."""

import numpy as np
from spline_generation import (
    BezierSplineGenerator, 
    SplineVisualizer, 
    EquidistantPointGenerator
)
from velocity_profile import (
    PlanningPoint,
    VelocityProfileGenerator,
    VelocityProfileVisualizer
)

def main():
    # # Define robot constraints
    # ROBOT_CONSTRAINTS = {
    #     'v_max': 2.0,         # Maximum translational velocity (m/s)
    #     'omega_max': 1.0,     # Maximum rotational velocity (rad/s)
    #     'a_t_max': 0.7,       # Maximum translational acceleration (m/s²)
    #     'a_r_max': 0.4,       # Maximum rotational acceleration (rad/s²)
    #     'f_max': 40,          # Maximum centripetal force (N)
    #     'mass': 10,           # Robot mass (kg)
    #     't_react': 0.1,       # Robot reaction time (s)
    # }
    
    ROBOT_CONSTRAINTS = {
        'v_max': 2.0,         # Maximum translational velocity (m/s)
        'omega_max': 1.5,     # Maximum rotational velocity (rad/s)
        'a_t_max': 1.0,       # Maximum translational acceleration/deceleration (m/s²)
        'a_r_max': 1.0,       # Maximum rotational acceleration (rad/s²)
        'f_max': 40,         # Maximum centripetal force (N) 50N (for a robot with mass m = 10 kg and min turning radius r = 1 m. i.e f = m * v^2 / r and v = 2 m/s i.e max velocity)
        'mass': 10,           # Robot mass (kg)
        't_react': 0.1,       # Robot reaction time (s)
    }

    # Define waypoints
    waypoints = [
        np.array([13.7414, 8.32168]),
        np.array([12.8583, 8.97716]),
        np.array([12.2636, 9.41402]),
        np.array([11.6704, 9.77417]),
        np.array([11.2832, 9.89637]),
        np.array([10.5473, 9.99393])
    ]

    # 1. Generate Bezier spline
    spline_gen = BezierSplineGenerator(waypoints, tangent_factor=0.5)
    
    # Visualize spline (optional)
    SplineVisualizer.visualize_spline(spline_gen)
    
    # 2. Generate equidistant points
    point_gen = EquidistantPointGenerator(spline_gen)
    points, parameters, arc_lengths = point_gen.generate_points(num_points=300)
    
    # 3. Convert to planning points
    planning_points = []
    for point, param, arc_length in zip(points, parameters, arc_lengths):
        # Get curvature at this point
        segment_idx, local_t = param
        deriv1 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=1)
        deriv2 = spline_gen.get_derivative_at_parameter(segment_idx, local_t, order=2)
        
        # Calculate curvature: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        denominator = (deriv1[0]**2 + deriv1[1]**2)**(3/2)
        if denominator > 1e-10:
            curvature = (deriv1[0]*deriv2[1] - deriv1[1]*deriv2[0]) / denominator
        else:
            curvature = 0.0
        
        # Create planning point
        planning_point = PlanningPoint(
            position=point,
            curvature=curvature,
            arc_length=arc_length
        )
        planning_points.append(planning_point)
    
    # 4. Generate velocity profile
    vpg = VelocityProfileGenerator(ROBOT_CONSTRAINTS)
    profile_points = vpg.generate_velocity_profile(planning_points)
    
    # 5. Save data for debugging (optional)
    VelocityProfileVisualizer.save_to_csv(
        profile_points, 
        "trajectory_debug_data.csv"
    )
    
    # 6. Visualize results
    VelocityProfileVisualizer.plot_profile(profile_points)
    
    # 7. Print summary
    total_time = profile_points[-1].time
    total_distance = profile_points[-1].arc_length
    avg_velocity = total_distance / total_time
    
    print("\nTrajectory Summary:")
    print(f"Total time: {total_time:.2f} seconds")
    print(f"Total distance: {total_distance:.2f} meters")
    print(f"Average velocity: {avg_velocity:.2f} m/s")
    
    # 8. Verify profile
    if vpg.verify_velocity_profile(profile_points):
        print("\nVelocity profile satisfies all constraints")
    else:
        print("\nWarning: Velocity profile violates some constraints")
        
    # After velocity profile generation
    print("\nVelocity Profile Statistics:")
    velocities = [p.velocity for p in profile_points]
    print(f"Max velocity: {max(velocities):.2f} m/s")
    print(f"Min velocity: {min(velocities):.2f} m/s")
    print(f"Avg velocity: {sum(velocities)/len(velocities):.2f} m/s")

    # Print key points velocities
    print("\nKey Points Velocities:")
    print(f"Start velocity: {profile_points[0].velocity:.2f} m/s")
    print(f"Mid velocity: {profile_points[len(profile_points)//2].velocity:.2f} m/s")
    print(f"End velocity: {profile_points[-1].velocity:.2f} m/s")

    # Print acceleration changes between points
    print("\nSample Accelerations:")
    for i in range(1, min(5, len(profile_points))):
        delta_v = profile_points[i].velocity - profile_points[i-1].velocity
        delta_s = profile_points[i].arc_length - profile_points[i-1].arc_length
        accel = (delta_v**2) / (2*delta_s)
        print(f"Point {i} acceleration: {accel:.2f} m/s²")

if __name__ == "__main__":
    main()