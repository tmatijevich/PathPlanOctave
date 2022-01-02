# LibPathPlanOctave

This project contains the [PathPlan](https://github.com/tmatijevich/LibPathPlan) library and scripts to demonstrate its functionality. The PathPlan library is a set of kinematic functions for one-dimensional motion planning.

- Clone the repository: `git@github.com:tmatijevich/LibPathPlanOctave.git`
	- Alternative: `ssh://git@ssh.github.com:443/tmatijevich/LibPathPlanOctave.git`
	
## PathPlan (Octave)

This project's PathPlan library is written in MATLAB for quick plotting and testing capabilities. As an alternative to commercial software, [GNU Octave](https://www.gnu.org/software/octave/) is used as the software environment.

Assumptions
- Positive distance and velocity
- Symmetric acceleration and deceleration
- Infinite jerk

List of functions
- PathAcc
	- `a = PathAcc(dt, dx, v_0, v_f, v_min, v_max)`
	- Minimum acceleration to move in time over a distance
- PathTime
	- `dt = PathTime(dx, v_0, v_f, v_min, v_max, a)`
	- Minimum time to move with acceleration over a distance
- PathDist
	- `dx = PathDist(dt, v_0, v_f, v_min, v_max, a)`
	- Maximum distance to move with acceleration in time
- PathVel
	- `v = PathVel(dt, dx, v_0, v_f, v_min, v_max, a)`
	- Minimum velocity to move with acceleration in time over a distance
- PathTimeDiff
	- `dt_tilde = PathTimeDiff(dx, v_0, v_f, v_min, v_max, a)`
	- Difference in durations between fastest possible and slowest possible profiles
- PathAccInTime
	- `a = PathAccInTime(dt_tilde, dx, v_0, v_f, v_min, v_max)`
	- Minimum acceleration to achieve a move within a window of time
- PathAccRiseInTime
	- `a = PathAccRiseInTime(dt_tilde, dx, v_1, v_f, v_min, v_max)`
	- Minimum acceleration to achieve a move with rise in a window of time
- PathRoots
	- `r_1, r_2 = PathRoots(p_2, p_1, p_0)`
	- Real roots of a second order polynomial
- PathPoint
	- `x, v, a = PathPoint(x_0, t_[n], v_[n], n, t)`
	- Position, velocity, and acceleration at a point in time along a linear-interpolated velocity profile
	
## Sample Plots

This library describes movement profiles in terms of velocity and time, where position and acceleration can be differentiated and integrated. The velocity profile has an initial and final velocity, plus an intermediate velocity saturated by the limits.

![2021-05-18_15 08 06](https://user-images.githubusercontent.com/33841634/118716809-de048300-b7ea-11eb-8022-8f65cd71a55c.png)

Provided a velocity point profile, the `PathPoint()` function will return the position, velocity, and acceleration at an intermediate time value.

![2021-05-18_15 10 21](https://user-images.githubusercontent.com/33841634/118717067-2e7be080-b7eb-11eb-86ff-90735e557f76.png)

`PathAccRiseInTime` finds the acceleration for two profiles to traverse a distance with a specified difference in time durations.

![2021-05-18_15 17 41](https://user-images.githubusercontent.com/33841634/118717859-36885000-b7ec-11eb-9d88-324cf8796e31.png)
