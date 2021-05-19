# LibPathPlanOctave

This repository contains functions and scripts to demonstrate the PathPlan library functions. Written in MATLAB, this repository is intended for use with [Octave](https://www.gnu.org/software/octave/index). The repository also serves the development of the PathPlan library.

- Clone the repository: `git@github.com:tmatijevich/LibPathPlanOctave.git`
	- Alternative: `ssh://git@ssh.github.com:443/tmatijevich/LibPathPlanOctave.git`
	
## PathPlan (Octave)

Developing the PathPlan library in Octave utilizes the easy plotting and brute-testing capabilities.

This library assumes:
- Positive distance and velocity.
- Symmetric acceleration and deceleration.
- Zero jerk.

List of functions:
- PathAcc
	- `a = PathAcc(dt, dx, v_0, v_f, v_min, v_max)`
	- Determine the minimum acceleration to move in time over a distance.
- PathTime
	- `dt = PathTime(dx, v_0, v_f, v_min, v_max, a)`
	- Determine the minimum time to move with acceleration over a distance.
- PathDist
	- `dx = PathDist(dt, v_0, v_f, v_min, v_max, a)`
	- Determine the maximum distance to move with acceleration in time.
- PathVel
	- `v = PathVel(dt, dx, v_0, v_f, v_min, v_max, a)`
	- Determine the minimum intermediate velocity to move with acceleration in time over a distance.
- PathTimeDiff
	- `tdiff = PathTimeDiff(dx, v_0, v_f, v_min, v_max, a)`
	- Determine the difference between the time minimizing and time maximizing velocity profiles.
- PathAccInTimeDiff
	- `a = PathAccInTimeDiff(dt_tilde, dx, v_0, v_f, v_min, v_max)`
	- Determine the minimum acceleration required to achieve movement extremes within a given time difference.
- PathAccInTimeDiffWithRise
	- `a = PathAccInTimeDiffWithRise(dt_tilde, dx, v_1, v_f, v_min, v_max)`
	- Same as `PathAccInTimeDiff`, but also consider an initial rise in velocity from standstill.
- PathRoots
	- `r1, r2 = PathRoots(p_2, p_1, p_0)`
	- Determine the real roots of a second order polynomial (quadratic equation) given the coefficients `p_2 * x^2 + p_1 * x + p_0 = 0`.
- PathPoint
	- `x, v, a = PathPoint(x0, t_[i], v_[i], n, t)`
	- Determine the position, velocity, and acceleration at a time point along a velocity point profile.
	
## Sample Plots

This library describes movement profiles in terms of velocity and time, where position and acceleration can be differentiated and integrated. The velocity profile has an initial and final velocity, plus an intermediate velocity saturated by the limits.

The `PathAcc()` function determines the minimum acceleration of a linear motion profile given the time, distance, initial & final velocities, and velocity limits.

Provided a velocity point profile, the `PathPoint()` function will return the position, velocity, and acceleration at an intermediate time value.

`PathAccInTimeDiffWithRise` finds the acceleration for two profiles to traverse a distance with a specified difference in time durations.

