```plaintext
***************************************************************************************************
==================================DO NOT EDIT/REMOVE CONTENT=======================================
==================================DRAFTED BY ABHISHEK NANNURI======================================
***************************************************************************************************
```

```mermaid
graph TD
    %% Inputs
    TRAJ[Trajectory Buffer] -->|"Trajectory (x,y,v,θ,t)"| BC[Base Controller Node]
    LOC[Localization Node] -->|Current Robot Pose| BC
    
    %% Controller Pipeline
    subgraph Controller_Package["Controller Package (amr_controller)"]
        style Controller_Package fill:#A7DCA6
        BC -->|"cmd_vel (linear/angular)"| MC[Motor Controller]
        MC -->|Motor Commands| RB[Robot Base]
        
        %% Internal Feedback
        RB -->|Encoder Feedback| MC
        MC -->|Status| BC
    end

    %% External Feedback
    RB -->|Odometry| LOC
    
    %% Configuration
    PARAMS[Parameter Server] -->|Controller Config| BC

    %% Debug/Monitoring
    BC -->|Controller Status| VIZ[RViz Visualization]
    BC -->|Controller Metrics| MON[Controller Monitor]

    style BC fill:#34A853,stroke:#000,stroke-width:2px

```