
```mermaid
flowchart TB
    subgraph ConfigSetup["1-Configuration Setup"]
        direction TB
        A[Config Files] --> B[Initial Robot Pose]
        A --> C[Goal Position]
        A --> D[Visualization Parameters]
    end
    style ConfigSetup fill:#1e3a8a,stroke:#60a5fa,color:#fff

    subgraph LocalizationNode["2-Localization Node"]
        direction TB
        E[Read Initial Pose] --> F[Publish to robot_pose]
        G[Subscribe to trajectory] --> H[Update Robot Position]
        H --> F
    end
    style LocalizationNode fill:#831843,stroke:#fb7185,color:#fff

    subgraph LocalPlanner["3-Local Planner Node"]
        direction TB
        I[Subscribe to robot_pose] --> J[Request Global Path]
        J --> K[Receive Lanelet IDs]
        K --> L[Process Lanelets]
    end
    style LocalPlanner fill:#064e3b,stroke:#34d399,color:#fff

    subgraph VizProcessing["4-Visualization Processing"]
        direction TB
        L --> M[Extract Left/Right Boundaries]
        L --> N[Compute Centerline]
        N --> O[Generate Smooth Path]
        O --> P[Generate Trajectory Points]
    end
    style VizProcessing fill:#783c00,stroke:#f97316,color:#fff

    subgraph MarkerPublisher["5-Marker Publisher"]
        direction TB
        subgraph BoundaryMarkers["Boundary Markers"]
            Q[Create Left Boundary] --> W[Publish to lanelet_boundaries]
            R[Create Right Boundary] --> W
        end
        subgraph PathMarkers["Path Markers"]
            S[Create Centerline] --> X[Publish to centerline]
            T[Create Smooth Path] --> Y[Publish to smooth_path]
        end
        subgraph TrajectoryMarkers["Trajectory Markers"]
            U[Create Trajectory Points] --> Z[Publish to trajectory]
            V[Create Moving Window] --> AA[Publish to moving_window]
        end
    end
    style MarkerPublisher fill:#581c87,stroke:#a855f7,color:#fff
    style BoundaryMarkers fill:#7f1d1d,stroke:#f87171,color:#fff
    style PathMarkers fill:#064e3b,stroke:#4ade80,color:#fff
    style TrajectoryMarkers fill:#1e3a8a,stroke:#818cf8,color:#fff

    M --> BoundaryMarkers
    N --> PathMarkers
    O --> PathMarkers
    P --> TrajectoryMarkers

    style A fill:#2563eb,color:#fff
    style B fill:#2563eb,color:#fff
    style C fill:#2563eb,color:#fff
    style D fill:#2563eb,color:#fff
    style E fill:#be185d,color:#fff
    style F fill:#be185d,color:#fff
    style G fill:#be185d,color:#fff
    style H fill:#be185d,color:#fff
    style I fill:#059669,color:#fff
    style J fill:#059669,color:#fff
    style K fill:#059669,color:#fff
    style L fill:#059669,color:#fff
    style M fill:#ea580c,color:#fff
    style N fill:#ea580c,color:#fff
    style O fill:#ea580c,color:#fff
    style P fill:#ea580c,color:#fff
    style Q fill:#dc2626,color:#fff
    style R fill:#dc2626,color:#fff
    style S fill:#059669,color:#fff
    style T fill:#059669,color:#fff
    style U fill:#3730a3,color:#fff
    style V fill:#3730a3,color:#fff
    style W fill:#dc2626,color:#fff
    style X fill:#059669,color:#fff
    style Y fill:#059669,color:#fff
    style Z fill:#3730a3,color:#fff
    style AA fill:#3730a3,color:#fff
```


1. **Configuration Setup** (Blue Section)
   - Starts with loading configuration files that contain:
     - Initial robot's position (for simulation)
     - Goal position for path planning
     - Visualization parameters (colors, sizes, update frequencies)

2. **Localization Node** (Pink Section)
   - Two main functions:
     - Reads initial robot pose and publishes it to `/robot_pose`
     - Subscribes to `/trajectory` topic to simulate robot movement
     - Continuously updates and republishes robot position based on trajectory

3. **Local Planner Node** (Green Section)
   - Main planning pipeline:
     - Subscribes to `/robot_pose` to get current position
     - Makes request to global planner with current and goal positions
     - Receives back a list of Lanelet IDs
     - Processes these Lanelets for local planning

4. **Visualization Processing** (Orange Section)
   - Takes processed Lanelets and:
     - Extracts left/right boundary points
     - Computes centerline from boundaries
     - Generates smooth path using centerline
     - Creates trajectory points for robot to follow

5. **Marker Publisher** (Purple Section)
   Has three main subsystems:
   
   a. **Boundary Markers** (Light Red)
   - Creates and publishes left/right boundary visualizations
   - Publishes to `/lanelet_boundaries` topic

   b. **Path Markers** (Light Green)
   - Handles centerline and smooth path visualization
   - Publishes to separate topics:
     - `/centerline` for raw centerline
     - `/smooth_path` for smoothed path

   c. **Trajectory Markers** (Light Blue)
   - Visualizes trajectory points and moving window
   - Publishes to:
     - `/trajectory` for trajectory points
     - `/moving_window` for local planning window

The data flows from top to bottom, with each section building upon the previous ones:
1. Config provides initial setup
2. Localization simulates robot position
3. Local planner processes this position
4. Visualization processing creates displayable data
5. Marker publisher sends everything to RViz
