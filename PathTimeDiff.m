%!octave

function [solution, valid] = PathTimeDiff(dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Determine the differnce between the time minimizing and time maximizing velocity profiles
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-03-25
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	solution.accDec.v = [0.0, 0.0, 0.0, 0.0];
	solution.accDec.t = [0.0, 0.0, 0.0, 0.0];
	solution.accDec.move = PATH_MOVE_NONE;
	solution.decAcc.v = [0.0, 0.0, 0.0, 0.0];
	solution.decAcc.t = [0.0, 0.0, 0.0, 0.0];
	solution.decAcc.move = PATH_MOVE_NONE;
	solution.dt_tilde = 0.0;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (v_min <= 0.0) || (v_max <= v_min) % *** Requires non-zero v_min (& v_max)
		printf("PathTimeDiff call failed: Implausible velocity limits %1.3f, %1.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < v_min) || (v_0 > v_max) || (v_f < v_min) || (v_f > v_max) % *** Thus requires non-zero endpoint velocities
		printf("PathTimeDiff call failed: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive distance and acceleration
	elseif (dx <= 0.0) || (a <= 0.0)
		printf("PathTimeDiff call failed: Distance or acceleration non-positive %1.3f, %1.3f\n", dx, a); 
		valid = false; 
		return;
		
	% #4: Valid distance given acceleration
	elseif dx < (abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a))
		printf("PathTimeDiff call failed: Implausible distance %1.3f given minimum %1.3f\n", dx, abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a)); 
		valid = false; 
		return;
		
	end
	
	% Determine the time minimizing profile
	dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	if dx < dx_u % Acc/dec profile with peak
		solution.accDec.move = PATH_ACC_DEC_PEAK;
		
		% Determine the peak velocity
		solution.accDec.v(2) = sqrt(dx * a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		solution.accDec.v(3) = solution.accDec.v(2);
		solution.accDec.t(2) = (solution.accDec.v(2) - v_0) / a;
		solution.accDec.t(3) = (solution.accDec.v(2) - v_0) / a;
		solution.accDec.t(4) = (solution.accDec.v(2) - v_0) / a + (solution.accDec.v(3) - v_f) / a;
		
	else % Acc/dec profile saturated at v_max
		solution.accDec.move = PATH_ACC_DEC_SATURATED;
		
		% Determine time at set velocity
		dt_12 = (dx - dx_u) / v_max;
		solution.accDec.v(3) = v_max;
		solution.accDec.v(2) = v_max;
		solution.accDec.t(2) = (v_max - v_0) / a;
		solution.accDec.t(3) = (v_max - v_0) / a + dt_12;
		solution.accDec.t(4) = (v_max - v_0) / a + dt_12 + (v_max - v_f) / a;
		
	end % Vmax distance threshold?
	
	% Determine the time maximizing profile
	dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a);
	if dx < dx_l % Dec/acc profile with dip
		solution.decAcc.move = PATH_DEC_ACC_PEAK;
		
		% Determine the dip velocity
		solution.decAcc.v(2) = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * a);
		solution.decAcc.v(3) = solution.decAcc.v(2);
		solution.decAcc.t(2) = (v_0 - solution.decAcc.v(2)) / a;
		solution.decAcc.t(3) = (v_0 - solution.decAcc.v(2)) / a;
		solution.decAcc.t(4) = (v_0 - solution.decAcc.v(2)) / a + (v_f - solution.decAcc.v(3)) / a;
		
	else % Dec/acc profile saturated at v_min
		solution.decAcc.move = PATH_DEC_ACC_SATURATED;
		
		% Determine the time at set velocity
		dt_12 = (dx - dx_l) / v_min;
		solution.decAcc.v(2) = v_min;
		solution.decAcc.v(3) = v_min;
		solution.decAcc.t(2) = (v_0 - v_min) / a;
		solution.decAcc.t(3) = (v_0 - v_min) / a + dt_12;
		solution.decAcc.t(4) = (v_0 - v_min) / a + dt_12 + (v_f - v_min) / a;
		
	end % Vmin distance threshold?
	
	% Set solution
	solution.accDec.v(1) = v_0;
	solution.accDec.v(4) = v_f;
	solution.decAcc.v(1) = v_0;
	solution.decAcc.v(4) = v_f;
	solution.dt_tilde = solution.decAcc.t(4) - solution.accDec.t(4);
	valid = true;
	
	if printResult
		printf("PathTimeDiff call: Time diff %.3f - %.3f = %.3f, Moves %d, %d\n", solution.decAcc.t(4), solution.accDec.t(4), solution.dt_tilde, solution.accDec.move, solution.decAcc.move);
	end
	
end % Function
