```plaintext
***************************************************************************************************
==================================DO NOT EDIT/REMOVE CONTENT=======================================
==================================DRAFTED BY ABHISHEK NANNURI======================================
***************************************************************************************************
```

                        ============================================================================
                        ===========================Global Path Planner==============================
                        ============================================================================

```mermaid

graph TD
    %% Input from Map Server to AMR Nav Sys PKG
    A[Map Server] --> |Static .OSM map| B[AMR Nav Sys PKG]
    
    %% Local Planner Request
    C[Local Planner PKG] --> |Client Req: start + End GPS| B 

    %% AMR Nav Sys Pipeline
    subgraph AMR_NavSys_Pipeline["AMR Nav Sys Pipeline"]
        style AMR_NavSys_Pipeline fill:#AECBFA
        GPP[Global Path Planner Server] --> |Load Map Data| GM[Graph Builder]
        GM --> |Build Graph| OP[Optimal Path Planner]
        OP --> |Find Optimal Path| CR[Construct Response]
        CR --> |Construct Lanelet/Area IDs Vector| RETURN[Send Response]
    end

    %% Response from AMR Nav Sys PKG to Local Planner
    RETURN --> |Server Res: Vector of Lanelet/Area IDs| C

    %% Connect Pipeline to AMR Nav Sys PKG
    B -->|Internal Flow| GPP

```

**Map** = HS-SchmalkaldenPart1.osm

**Client Request:**
- ```plaintext
    # Request
    float64 start_latitude
    float64 start_longitude
    float64 start_altitude    # Default altitude is 0.0
    float64 end_latitude
    float64 end_longitude
    float64 end_altitude      # Default altitude is 0.0
    bool use_time_based_routing
    ```
**Server Response:**
- ```plaintext
    # Response
    int64[] lanelet_ids      # Sequence of IDs (Lanelet or Area or Both) forming the Optimal path
    bool[] is_inverted        # Boolean array indicating if the corresponding lanelet is inverted
    float64 total_distance     # Estimated total distance of the path in meters
    float64 estimated_time     # Estimated time to traverse the path in seconds
    uint8 status              # Status code (0 for success, non-zero for errors)
    string message            # Additional information or error message
    ```
