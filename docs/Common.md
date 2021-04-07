# Common Equations

## Time & Distance

Nearly every function in this library is derived from the equation defining time and/or distance of a velocity profile segment.

- The *time duration* $\delta t$ of a velocity segment is simply:

$$
\delta t = \frac{|v_0 - v_f|}{a}
$$

- The *distance* $\delta x$ is:

$$
\delta x = \frac{|v_0^2 - v_f^2|}{2a}
$$

- This octave plot depicts the variables. 

![Profile segment plot](./media/VelocitySegment.png){: .image-center }

!!! note "PathPlan: $a > 0$"
	In this library's implementation, *acceleration* $a$ is defined as positive representing the magnitude of acceleration or deceleration of a velocity profile segment. (Therefore a single acceleration magnitude for a multi-segment velocity profile with symmetric acceleration/deceleration.)

## Derivation

