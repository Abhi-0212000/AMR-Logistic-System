```sh
ros2 action send_goal --feedback /navigate_to_goal amr_interfaces/action/NavigateToGoal "{
  goal_gps: [50.7154152, 10.4678595, 0.0],
  lanelet_ids: [-6778, -6802, -6779, -6782, -6768, -6788, -6793, -6777, -6784, -6776],
  is_inverted: [False, False, False, False, False, False, False, True, False, False],
  total_distance: 147.2242238312887,
  estimated_time: 9.054729203596866
}"
```