%!octave

function [solution, valid] = PathTime(dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% function [solution, valid] = PathTime(dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Determine the minimum time to change velocity with acceleration over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-23
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
		printf("PathTime call failed: Implausible velocity limits %1.3f, %1.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < v_min) || (v_0 > v_max) || (v_f < v_min) || (v_f > v_max)
		printf("PathTime call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive distance and acceleration
	elseif (dx <= 0.0) || (a <= 0.0)
		printf("PathTime call failed: Distance or acceleration non-positive %1.3f, %1.3f\n", dx, a); 
		valid = false; 
		return;
		
	% #4: valid distance given acceleration
	elseif dx < (abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a))
		printf("PathTime call failed: Implausible distance %1.3f given minimum %1.3f\n", dx, abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a)); 
		valid = false; 
		return;
		
	end % Requirements
	
	% There is no time advantage to decelerating below the final velocity, therefore only ACC_DEC profile will be considered
	dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	
	% Check if saturated profile
	if dx < dx_u % Acc/dec profile with peak
		solution.move = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		v_peak = sqrt(dx * a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		
		% Set the solution
		solution.t(2) = (v_peak - v_0) / a;
		solution.t(3) = (v_peak - v_0) / a;
		solution.t(4) = (v_peak - v_0) / a + (v_peak - v_f) / a;
		solution.v(2) = v_peak;
		solution.v(3) = v_peak;
		
	else % Acc/dec profile saturated at v_max
		solution.move = PATH_ACC_DEC_SATURATED;
		
		% Determine the time at v_max
		t_12 = (dx - dx_u) / v_max;
		
		% Set the solution
		solution.t(2) = (v_max - v_0) / a;
		solution.t(3) = (v_max - v_0) / a + t_12;
		solution.t(4) = (v_max - v_0) / a + t_12 + (v_max - v_f) / a;
		solution.v(2) = v_max;
		solution.v(3) = v_max;
		
	end % Profile type
	
	% Set common solution values and validate
	solution.dx = dx;
	solution.v(1) = v_0;
	solution.v(4) = v_f;
	solution.a = a;
	valid = true;
	
	if printResult
		printf("PathTime call: Time %1.3f, Vel %1.3f, Move %d\n", solution.t(4), solution.v(2), solution.move);
	end
	
end % Function
