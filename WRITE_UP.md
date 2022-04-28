## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

###### Explanation

After arming the drone, the `MotionPlanning.plan_path()` takes control. 
Here, the code does the following:

1. We set a fixed altitude of 5m to the drone's trajectory.
2. We obtain a grid representation of the map stored in `colliders.csv` using `planning_utils.create_grid()`.
This method returns a numpy array, where each available cell has the value of `0.0` and an unreachable cell
has the value of `1.0` considering the drone's altitude (see 1.) and a safety distance regarding 
obstacles (with value 5m).
3. We set the drone's goal destination, as the cell positioned 10m north and 10m east from the starting
location.
4. The function `planning_utils.a_star()` produces a plan for reaching the goal destination (see 3.)
via the A* algorithm using the euclidean distance as heuristic.
This plan is composed by a list of waypoints.

The plan is stored at the attribute `waypoints`. Then, by calling `MotionPlanning.send_waypoints()`
we trigger takeoff and then waypoint traversal, in a similar fashion as in
the Backyard flier project.
Once all the waypoints have been reached (within a 1m range as defined in `MotionPlanning.local_position_callback()`)
the drone proceed to land and disarm.


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

###### Explanation 
At the method `MotionPlanning.set_global_home()`, we do the following:

1. We read the first line of the `map_file` text file.
2. We split this line first using a `,` separator.
3. For each one of the strings obtain in 2. we split them using the `<space>` separator.
The second resulting string corresponds to the numeric values we need.

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

###### Explanation 
At the method `MotionPlanning.determine_local_position()`, we obtain the global
position accessing the attributes `self._longitude`, `self._latitude`
and `self._altitude`.
Then, we pass these value to `frameutils.global_to_local()` to obtain a 
local position relative to `self.global_home`.

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

###### Explanation 

In `MotionPlanning.get_starting_location()`, we convert the 
current position, available at `self.local_position` to a valid
grid value.
We accomplish this by subtracting the offsets produced by
`planning_utils.create_grid()`.

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

###### Explanation 
We obtain the goal grid location from longitude and latitude
values in `MotionPlanning.get_goal_location()`.
The method does the following:

1. We use `frameutils.global_to_local()` to convert the longitude and latitude
values to a local position, relative to `self.global_home.`
2. We subtract the offsets returned from `planning_utils.create_grid()` to
generate valid grid values from the local position obtained in 1.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

###### Explanation 
To enable diagonal motion, we modify these code elements:

1. The `Action` enumeration: We added four elements corresponding
to `NORTH_WEST`, `NORTH_EAST`, `SOUTH_WEST` and `SOUTH_EAST` actions.
As per the instructions, the cost of these actions is `sqrt(2)`.
2. The `planning_utils.valid_actions()` function: We added extra code
to remove our brand-new actions in case they move the drone
to unfeasible positions, like collision with obstacles or outside 
map limits.


#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

###### Explanation 

We implemented the pruning logic in the `motion_planning.prune_path()` method.
It does the following:

1. Given a path generated using `planing_utils.a_star()`, it checks if
three consecutive waypoints are collinear using `motion_planning.check_collinearity()`.
2. If they are collinear, one of the waypoints is removed.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


