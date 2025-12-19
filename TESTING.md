# Testing robot turning and movement
Testing sequence:
1. Run launch file  
2. Run client  
3. For every point  
   3.1 Send point   
   3.2 Wait until the robot stops   

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