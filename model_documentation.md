
# The following parts are to generate path to drive the vehicle

## Vehicle Positioning

Vehicle positioning ensures that the vehicle stays far enough back from the car in front, as well as not hitting a vehicle to the left or right lanes when it decides to make a lane change. 
### Lane Change
1. Find vehicle that is in front of ego_vehicle (same lane, ahead of ego vehicle within 30m).
2. Set too_close flag, which triggers either slowing down or initiating a lane change.
3. Since we want to maintain high speed, lane change is more desirable. I decide to prioritize a left lane change over right lane.
4. Sort all vehicles into vehicles in the lane to the left and vehicles in the lane to the right.
5. Check left vehicles first since we want to make left lane change.
6. If there are no cars within 30m in front or behind the s position of our car, the lane variable is changed to the lane to the left. If this is not possible, then the right lane is checked for an opening. If so, the lane variable is changed to the lane to the right. If this is also not possible, the lane is not changed. 

## Speed Adjusting

Adjust the speed. If it's been determined that our vehicle is too close to the vehicle ahead, then the reference velocity is decreased by ~5m/s^2. Otherwise, if the speed is less than our desired speed of 49.5m/s, then the reference velocity is increased by ~5m/s^2.

## Missing Points

Next, we need to generate the missing points in the path to go out 30m in front of ego car. If the size of the path is less than 2 points, then the first two are filled out with the last point and the current point of the coarse lane points. Otherwise, the first two points are filled out based on the previous path. Next, three more points are added along the coarse lane centerline for 90 meters. Next the points are shifted to be in the frame of the vehicle.

## Spline Generation

Next, a spline is generated using the coarse path, and then x and y values are generated from the spline to 30 meters out. points along this spline are added to the end of the current smooth path, making the total length 50 points.
