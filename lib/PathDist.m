%!octave

function [solution, valid] = PathDist(dt, v_0, v_f, v_min, v_max, a, printResult = false)
	% function [solution, valid] = PathDist(dt, v_0, v_f, v_min, v_max, a, printResult = false)
	% Determine the maximum distance to change velocity with acceleration in time
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-29
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	solution.t = [0.0, 0.0, 0.0, 0.0];
	solution.dx = 0.0;
	solution.v = [0.0, 0.0, 0.0, 0.0];
	solution.a = 0.0;
	solution.move = PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (v_min < 0.0) || (v_max <= v_min)
		printf("PathDist call failed: Implausible velocity limits %1.3f, %1.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < v_min) || (v_0 > v_max) || (v_f < v_min) || (v_f > v_max)
		printf("PathDist call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive timespan and acceleration
	elseif (dt <= 0.0) || (a <= 0.0)
		printf("PathDist call failed: Timespan or acceleration non-positive %1.3f, %1.3f\n", dt, a); 
		valid = false; 
		return;
		
	% #4: Valid timespan given acceleration
	elseif dt < (abs(v_0 - v_f) / a)
		printf("PathDist call failed: Impossible timepsan %1.3f given minimum %1.3f\n", dt, abs(v_0 - v_f) / a); 
		valid = false; 
		return;
		
	end % Requirements
	
	% There is no distance advantage to decelerate then accelerate, therefore only ACC_DEC profiles will be considered
	dt_u = (2 * v_max - v_0 - v_f) / a;
	
	% Check if saturated profile
	if dt < dt_u % Acc/dec profile with peak
		solution.move = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		v_p = (dt * a + v_0 + v_f) / 2.0;
		
		% Set the solution
		solution.dx = (2.0 * v_p ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
		solution.t(2) = (v_p - v_0) / a;
		solution.t(3) = (v_p - v_0) / a;
		solution.v(2) = v_p;
		solution.v(3) = v_p;
		
	else % Acc/dec profile saturated at v_max
		solution.move = PATH_ACC_DEC_SATURATED;
		
		% Determine the distance at v_max
		dx_12 = (dt - (2.0 * v_max - v_0 - v_f) / a) * v_max;
		
		% Set the solution
		solution.dx = dx_12 + (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
		solution.t(2) = (v_max - v_0) / a;
		solution.t(3) = (v_max - v_0) / a + dx_12 / v_max;
		solution.v(2) = v_max;
		solution.v(3) = v_max;
		
	end
	
	% Set common solution values and validate
	solution.t(4) = dt;
	solution.v(1) = v_0;
	solution.v(4) = v_f;
	solution.a = a;
	valid = true;
	
	if printResult
		printf("PathDist call: Dist %.3f, Vel %.3f, Move %d\n", solution.dx, solution.v(2), solution.move);
	end
	
end
