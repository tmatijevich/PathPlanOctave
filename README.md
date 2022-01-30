# PathPlan (Octave)

PathPlan is a library of kinematic functions for one-dimensional motion planning. The functions are lightweight to solve for motion profiles in real-time. Following the linear segments with parabolic blends (LSPB) method, the motion profiles are simple yet smooth between points. An optional jerk factor brings the motion profile up to the 3rd-order (see PathPoint).

Written in MATLAB, this library uses the scientific programming platform for its scripting and plotting capabilities. As an alternative to commercial software, [GNU Octave](https://www.gnu.org/software/octave/) can be used.

Clone: `git@github.com:tmatijevich/PathPlanOctave.git`
- Alternative: `ssh://git@ssh.github.com:443/tmatijevich/PathPlanOctave.git`

Assumptions:

- Positive distance and velocity
- Symmetric acceleration and deceleration
- Infinite jerk

List of functions:

- PathAcc
	- `a = PathAcc(dt, dx, v_0, v_f, v_min, v_max)`
	- Minimum acceleration to move in time over a distance
- PathTime
	- `dt = PathTime(dx, v_0, v_f, v_min, v_max, a)`
	- Minimum time to move with acceleration over a distance
- PathDist
	- `dx = PathDist(dt, v_0, v_f, v_min, v_max, a)`
	- Maximum distance from move with acceleration in time
- PathVel
	- `v = PathVel(dt, dx, v_0, v_f, v_min, v_max, a)`
	- Minimum velocity to move with acceleration in time over a distance
- PathTimeDiff
	- `dt_tilde = PathTimeDiff(dx, v_0, v_f, v_min, v_max, a)`
	- Difference in time durations between fastest and slowest moves
- PathAccInTime
	- `a = PathAccInTime(dt_tilde, dx, v_0, v_f, v_min, v_max)`
	- Minimum acceleration to complete any move in a window of time
- PathAccRiseInTime
	- `a = PathAccRiseInTime(dt_tilde, dx, v_1, v_f, v_min, v_max)`
	- Minimum acceleration to complete any move with rise in a window of time
- PathRoots
	- `r_1, r_2 = PathRoots(p_2, p_1, p_0)`
	- Real roots of a second order polynomial
- PathPoint
	- `x, v, a = PathPoint(x_0, t_[n], v_[n], n, t)`
	- Position, velocity, and acceleration at a point in time along linear segments with parabolic blends
	
## Sample Plots

![2021-05-18_15 08 06](https://user-images.githubusercontent.com/33841634/118716809-de048300-b7ea-11eb-8022-8f65cd71a55c.png)

![2021-05-18_15 10 21](https://user-images.githubusercontent.com/33841634/118717067-2e7be080-b7eb-11eb-86ff-90735e557f76.png)

![2021-05-18_15 17 41](https://user-images.githubusercontent.com/33841634/118717859-36885000-b7ec-11eb-9d88-324cf8796e31.png)
