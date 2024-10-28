# Motion Planning Approaches Comparison

| **Approach** | **Planner Type** | **Dynamic Obstacles Handling** | **Main Use Case** | **Key Steps** | **Output** | **Advantages** | **Disadvantages** | **Computational Cost** |
|--------------|------------------|-------------------------------|-------------------|---------------|------------|---------------|------------------|---------------------|
| **Moving Window + Centerline** | Local Planner (Geometric) | Limited (focus on static environment) | Static environment navigation | 1. Generate moving window<br>2. Extract centerline<br>3. Apply spline smoothing<br>4. Generate trajectory<br>5. Output `cmd_vel` | Trajectory points (x, y, v, θ, t) | - Simple and modular<br>- Low computational cost<br>- Easy to implement | - No dynamic obstacle avoidance yet<br>- Basic obstacle handling | Low |
| **DWA** (Dynamic Window) | Local Planner (Reactive) | Real-time dynamic obstacle avoidance | Quick, reactive local navigation | 1. Predict velocities<br>2. Evaluate constraints<br>3. Select best velocity<br>4. Output `cmd_vel` | Direct `cmd_vel` commands | - Fast real-time execution<br>- Low memory usage<br>- Good reactive behavior | - Non-optimal paths<br>- Local minima issues | Low-Medium |
| **TEB** (Timed Elastic Band) | Local Planner (Optimization) | Continuous trajectory optimization | Optimal path planning with dynamics | 1. Optimize trajectory<br>2. Consider constraints<br>3. Output smooth `cmd_vel` | Optimized `cmd_vel` | - Smooth paths<br>- Considers kinematics<br>- Good obstacle avoidance | - Complex tuning<br>- Computationally heavy | High |
| **Lattice Planners** | Global/Local (Structured) | Pre-planned avoidance | Structured environments | 1. Discretize space<br>2. Generate primitives<br>3. Search optimal path | Feasible trajectory | - Predictable<br>- Good for structured spaces | - High memory usage<br>- Not ideal for dynamics | Medium-High |
| **RRT** (Rapidly Exploring Random Trees) | Global/Local (Sampling) | Sampling-based avoidance | Complex environments | 1. Random sampling<br>2. Tree expansion<br>3. Path refinement | Path trajectory | - Handles complexity well<br>- Probabilistically complete | - Non-optimal paths<br>- Variable computation time | Medium-High |
| **APF** (Artificial Potential Field) | Local Planner (Reactive) | Simple real-time avoidance | Basic reactive navigation | 1. Calculate forces<br>2. Generate motion | Direct `cmd_vel` | - Very simple<br>- Fast computation | - Local minima<br>- Oscillations | Low |
| **MPC** (Model Predictive Control) | Local Planner (Predictive) | Predictive avoidance | Precise tracking | 1. State prediction<br>2. Optimization<br>3. Control output | Optimal controls | - Precise control<br>- Predictive capability | - Complex model needed<br>- Computationally heavy | Very High |
| **Hybrid A\*** | Global/Local (Hybrid) | Combined static/dynamic | Complex navigation | 1. A* search<br>2. Continuous expansion<br>3. Path smoothing | Feasible path | - Complete solution<br>- Handles constraints | - Memory intensive<br>- Complex implementation | High |
| **State Lattice** | Global/Local (Structured) | Pre-computed responses | Structured dynamics | 1. Generate lattice<br>2. Connect states<br>3. Find optimal path | State trajectory | - Deterministic<br>- Good with constraints | - High preprocessing<br>- Memory intensive | Medium-High |

# Recommendations

## Current Scenario (Static Obstacles Only)
Best Approach: **Moving Window + Centerline** with potential enhancement to **APF**

Reasoning:
1. Your current Moving Window + Centerline approach is well-suited for static environments
2. It provides good performance with low computational overhead
3. Simple to implement and maintain
4. Easy to debug and tune

Implementation Strategy:
1. Keep using the current Moving Window + Centerline approach
2. Consider adding APF for basic obstacle avoidance if needed
3. Focus on optimizing the centerline extraction and smoothing
4. Implement robust error handling and failsafes

## Future Scenario (With Dynamic Obstacles)
Recommended Evolution Path:

### Phase 1: DWA Integration
Start with **DWA** because:
- Relatively simple to implement
- Good balance of performance and complexity
- Direct `cmd_vel` output
- Low computational overhead
- Natural extension of current approach

### Phase 2: TEB Implementation
Graduate to **TEB** when you need:
- Better path optimization
- Improved dynamic obstacle handling
- More sophisticated trajectory planning
- Can handle the additional computational cost

Implementation Strategy:
1. Start with DWA implementation while keeping current centerline extraction
2. Gradually introduce dynamic obstacle detection and tracking
3. Add reactive behaviors with DWA
4. Once stable, consider upgrading to TEB if needed

### Advanced Considerations:
- If your environment becomes more structured, consider Lattice Planners
- If you need precise control and have computational resources, consider MPC
- If you face very complex environments, consider RRT or Hybrid A*

Notes:
- Keep the current system as a fallback during integration
- Test thoroughly in simulation before deploying
- Consider computational resources when choosing approach
- Monitor performance metrics during transitions




# MPC Classification Clarification

MPC (Model Predictive Control) is primarily a control strategy rather than a pure motion planner. Here's the important distinction:

## MPC as a Controller
- Primary role: Generates optimal control inputs based on system model and constraints
- Works like an advanced version of PID with prediction capabilities
- Focuses on tracking reference trajectories
- Operates at the control level (managing actuator inputs)

## MPC in Motion Planning Context
Sometimes MPC is mentioned in planning discussions because it can be used in a hybrid planning-control approach:

1. **Traditional Planning-Control Setup:**
```
Motion Planner → Trajectory → Controller (PID) → Robot
```

2. **MPC-Enhanced Setup:**
```
Motion Planner → Reference Path → MPC Controller → Robot
```

3. **MPC as Planning-Control Fusion:**
```
Combined MPC (Planning + Control) → Robot
```

## Correct Classification

| Component | Primary Classification | Secondary Role |
|-----------|----------------------|----------------|
| MPC | Controller | Can incorporate some local planning aspects |
| PID | Controller | Pure control, no planning |
| DWA | Local Planner | May include simple control aspects |
| TEB | Local Planner | Includes trajectory optimization |

## Recommended Table Update
The previous table should be modified to remove MPC from the pure planners list and instead add a note about control strategies:

### Control Strategies (Separate from Planners)
| **Approach** | **Type** | **Purpose** | **Key Features** |
|--------------|----------|-------------|------------------|
| **PID** | Pure Controller | Trajectory tracking | - Simple implementation<br>- Well-understood behavior<br>- No predictive capability |
| **MPC** | Predictive Controller | Optimal control with prediction | - Model-based prediction<br>- Handles constraints<br>- Can incorporate obstacle avoidance |

## Key Distinctions
1. **Planning vs Control**
   - Planners: Generate desired paths/trajectories
   - Controllers: Generate control commands to follow paths

2. **MPC's Unique Position**
   - Primary role: Advanced controller
   - Secondary capability: Can incorporate some local planning aspects
   - Often used in conjunction with higher-level planners

3. **Integration Approach**
   - Best practice: Use MPC as a sophisticated controller
   - Pair with dedicated planners for complete navigation
   - Can handle some obstacle avoidance within control horizon

## Implementation Recommendation
For your system:
1. Keep motion planning (DWA/TEB/etc.) separate from control
2. Consider MPC as an alternative to PID for trajectory tracking
3. If using MPC, structure as:
   ```
   Global Planner → Local Planner (DWA/TEB) → MPC Controller → Robot
   ```