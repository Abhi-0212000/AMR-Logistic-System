```sh
ros2 service call /amr_goal_management amr_interfaces/srv/AmrGoalManagement \
'{goal_latitude: 50.7153508, goal_longitude: 10.4680332, goal_altitude: 0.0}'
```

launching node:
```sh
ros2 run amr_central_management amr_central_management_node
```