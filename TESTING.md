# Testing robot turning and movement  
## Automatic testing  
1. Run launch file by `ros2 launch wheeled_model_enhanced integration_test.launch.py`  
2. Wait until test will be over  
3. Results will be printed in console and in file named `integration_test.xml` in jest-junit format  

## Testing by hand  
Testing sequence:
1. Run launch file by `ros2 launch wheeled_model_enhanced wheeled_model_enhanced_simulation.launch.py`
2. Run client by `ros2 run wheeled_model_enhanced reach_goal_action_client`
3. For every point  
   3.1 Send point   
   3.2 Wait until the robot stops (otherwise the goal will be rejected)

Points to turn to (to place a waypoint on) in `{x, y}` and `lat long` in rads:
1. 10; 10 - turn to the right; (lat long) `-0.4011918 -0.75402419`
2. 10; 0 - to the left; (lat long) `-0.40119337 -0.75402419`
3. 10; -10 - to the left; (lat long) `-0.40119495 -0.75402419`
4. 0; -10 - to the left; (lat long) `-0.40119495 -0.75402589`
5. -10; -10 - to the left; (lat long) `-0.40119495 -0.75402759`
6. -10; 0 - no turning, just going forward; (lat long) `-0.40119337 -0.75402759`
7. -10; 10 - to the right; (lat long) `-0.4011918 -0.75402759`
8. 0; 10 - to the right; (lat long) `-0.4011918 -0.75402589`
9. -2; 0 - no movement due the goal is already reached; (lat long) `-0.40119337 -0.75402623`