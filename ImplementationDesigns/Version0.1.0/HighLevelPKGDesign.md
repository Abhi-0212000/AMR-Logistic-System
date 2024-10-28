```plaintext
***************************************************************************************************
==================================DRAFTED BY ABHISHEK NANNURI======================================
***************************************************************************************************
```

                        ============================================================================
                        ===================INITIAL HIGH-LEVEL DEVELOPMENT PLAN======================
                        =================Version 0.1.0 ---> No Dynamic Obstacles====================
                        ============================================================================

```mermaid

graph TD
    A[Global Path Planner] -->|Lanelet IDs| B[Local Path Planner]
    B -->|Trajectory Points| C[Motion Controller]
    G[Map Server] -->|Static Map| A
    G -->|Static Map| B
    D[Robot Localization] -->|Current Pose| B
    D -->|Current Pose| C
    C -->|Velocity Commands| E[Robot Base Controller]
    F[Robot State Publisher] -->|TF Tree| D

    style G fill:#0F9D58
    style A fill:#0F9D58
    style B fill: #008AD8

    %% Legend
    subgraph Legend
        V[Green: Developed]
        style V fill:#0F9D58
        W[Blue: Under Development]
        style W fill:#008AD8
    end
```

### Explanation:
- **Global Path Planner**: Provides the optimal path as a series of Lanelet IDs.
- **Local Path Planner**: Interprets the Lanelet IDs and generates a series of trajectory points for the local path.
- **Motion Controller**: Converts trajectory points into velocity commands that the AMR can follow.
- **Robot Localization**: Provides the current position (pose) of the AMR, which the local planner and motion controller use to correct the trajectory in real-time.
- **Robot Base Controller**: The final controller that sends the velocity commands to the robot’s actuators.
- **Robot State Publisher**: Publishes the robot's state (pose, transformations) to the ROS TF tree.
- **Map Server**: Provides the static map to the global path planner for route generation.

