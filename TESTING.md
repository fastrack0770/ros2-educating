# Testing robot turning and movement
Testing sequence:
1. Change the waypoint location
2. Run launch file
3. Run client
4. Fix the movement result

Points to turn to (to place a waypoint on) in `{x, y}`:
1. 10; 10 - turn to the right
2. 10; 0 - to the left
3. 10; -10 - to the left 
4. 0; -10 - to the left
5. -10; -10 - to the left
6. -10; 0 - no turning, just going forward
7. -10; 10 - to the right
8. 0; 10 - to the right
9. -2; 0 - no movement due the goal is already reached