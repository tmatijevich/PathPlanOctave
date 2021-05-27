# PathAcc Function

*Determine the minimum acceleration to change velocity in time over a distance.*

## `a = PathAcc(dt, dx, v_0, v_f, v_min, v_max)`


## Inputs

- `dt` Time duration
- `dx` Distance
- `v_0` Initial velocity
- `v_f` Final velocity
- `v_min` Minimum velocity
- `v_max` Maximum velocity

## Input Conditions

1. $\quad 0 \leq v_{min} < v_{max}$
2. $\quad v_{min} \leq v_0 \leq v_{max} \;,\; v_{min} \leq v_f \leq v_{max}$
3. $\quad \delta t \;,\; \delta x > 0$
4. $\quad v_{min} \cdot \delta t < \delta x < v_{max} \cdot \delta t$

## Derivation

Define a nominal distance, $\bar{x}$. 

- This is the distance if the acceleration magnitude matches the exact time to change velocity from $v_0$ to $v_f$.

$$
\bar{x} = \frac{1}{2} \delta t (v_0 + v_f)
$$

Given the time duration, determine the acceleration $a_u$ at the saturation limit of the velocity maximizing profile. 

- This is a two-segment velocity profile with symmetric acceleration magnitudes.
- $v_{max} \geq v_0, v_f$ from the input conditions.

$$
a_u = \frac{2 v_{max} - v_0 - v_f}{\delta t}
$$

Now determine the distance at the saturation limit of the velocity maximizing profile. 

$$
{\delta x}_u = \frac{2 v_{max}^2 - v_0^2 - v_f^2}{2 a_u}
$$

Re-written in terms of all inputs.

$$
{\delta x}_u = \frac{(2 v_{max}^2 - v_0^2 - v_f^2) \delta t}{2 v_{max} - v_0 - v_f}
$$

The same approach can be completed for the velocity minimizing profile.

$$
{\delta x}_l = \frac{(v_0^2 + v_f^2 - 2 v_{min}^2) \delta t}{v_0 + v_f - 2 v_{min}}
$$

The solution of the acceleration magnitude now comes down to four cases.

Case | Condition
-----|----------
`PATH_ACC_DEC_SATURATED` | $\bar{\delta x} \leq \delta x _u \leq \delta x$
`PATH_ACC_DEC_PEAK` | $\bar{\delta x} \leq \delta x < \delta x_u$
`PATH_DEC_ACC_PEAK` | $\delta x_l < \delta x < \bar{\delta x}$
`PATH_DEC_ACC_SATURATED` | $\delta x \leq \delta x_l < \bar{\delta x}$

1. The profile is saturated at $v_{max}$ for some time, $t_{12}$.

	$$
	\delta t = \frac{2 v_{max} - v_0 - v_f}{a} + t_{12} \quad \delta x = \frac{2 v_{max}^2 - v_0^2 - v_f^2}{2 a} + t_{12} v_{max}
	$$

	- Reduce the equations by eliminating $t_{12}$.

	$$
	\delta x = \frac{2 v_{max}^2 - v_0^2 - v_f^2}{2 a} + (\delta t - \frac{2 v_{max} - v_0 - v_f}{a}) v_{max}
	$$

	- Solve for $a$.

	$$
	a = \frac{2 v_{max}^2 - v_0^2 - v_f^2}{2 (\delta x - \delta t \, v_{max})} - \frac{2 v_{max} - v_0 - v_f}{\delta x - \delta t \, v_{max}}
	$$
	
2. The profile is unsaturated with a peak velocity, `PATH_ACC_DEC_SATURATED`.

	$$
	\delta t = \frac{2 v_{peak} - v_0 - v_f}{a} \quad \delta x = \frac{2 v_{peak}^2 - v_0 ^ 2 - v_f^2}{2 \, a}
	$$
	
	$$
	2 \delta x \frac{2 v_{peak} - v_0 - v_f}{\delta t} = 2 v_{peak}^2 - v_0^2 - v_f^2
	$$
	
	$$
	0 = \underbrace{2 \delta t}_{p_2} v_{peak}^2 + \underbrace{(-4 \delta x)}_{p_1} v_{peak} + \underbrace{2 \delta x (v_0 + v_f) - \delta t (v_0^2 + v_f^2)}_{p_0}
	$$
	
	Use the second order roots solution for $v_{peak}$, then solve for $a$.
	
	$$
	a = \frac{2 v_{peak} - v_0 - v_f}{\delta t}
	$$