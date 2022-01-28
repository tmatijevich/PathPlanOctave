%!octave

function [solution, valid] = PathVel(dt, dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% function [solution, valid] = PathVel(dt, dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Determine the minimum intermediate velocity to change velocity with acceleration 
	% in time over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-12-30
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_DEC_DEC;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	global PATH_ACC_ACC;
	
	% Reset solution
	solution.t = [0.0, 0.0, 0.0, 0.0];
	solution.dx = 0.0;
	solution.v = [0.0, 0.0, 0.0, 0.0];
	solution.a = 0.0;
	solution.move = PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (v_min < 0.0) || (v_max <= v_min)
		printf("PathVel call failed: Implausible velocity limits %.3f, %.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < v_min) || (v_0 > v_max) || (v_f < v_min) || (v_f > v_max)
		printf("PathVel call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive time, distance, and acceleration
	elseif (dt <= 0.0) || (dx <= 0.0) || (a <= 0.0)
		printf("PathVel call failed: Time, distance, or acceleration non-positive %.3f, %.3f, %.3f\n", dt, dx, a); 
		valid = false; 
		return;
		
	% #4: Valid time and distance given acceleration
	elseif (dt < abs(v_0 - v_f) / a) || (dx < abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a))
		printf("PathVel call failed: Impossible time %.3f or distance %.3f given limits %.3f, %.3f\n", dt, dx, abs(v_0 - v_f) / a, abs(v_0 ^ 2 - v_f ^ 2) / a);
		valid = false; 
		return;
		
	end % Requirements
	
	% Check if the distance can be fulfilled given the time duration, velocity limits, and acceleration
	v_p = (a * dt + v_0 + v_f) / 2.0;
	if v_p <= v_max
		dx_max = (2.0 * v_p ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	else
		dx_max = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a) + v_max * (dt - ((2.0 * v_max - v_0 - v_f) / a));
	end
	if dx > dx_max
		printf("PathVel call failed: Distance %.3f exceeds maximum %.3f\n", dx, dx_max);
		valid = false;
		return;
	end
	
	vdip = (v_0 + v_f - a * dt) / 2.0;
	if vdip >= v_min
		dx_min = (v_0 ^ 2 + v_f ^ 2 - 2.0 * vdip ^ 2) / (2.0 * a);
	else
		dx_min = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a) + v_min * (dt - ((v_0 + v_f - 2.0 * v_min) / a));
	end
	if dx < dx_min
		printf("PathVel call failed: Distance %.3f exceeds minimum %.3f\n", dx, dx_min);
		valid = false;
		return;
	end
	
	% Determine nominal distance which decide acceleration directions
	dt_bar = abs(v_0 - v_f) / a;
	dx_bar = abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	dx_a1 = v_0 * (dt - dt_bar) + dx_bar;
	dx_a2 = dx_bar + v_f * (dt - dt_bar);
	
	% Determine the sign of both acceleration phases
	if dx > dx_a1
		a1Sign = 1.0;
	elseif dx < dx_a1
		a1Sign = -1.0;
	else
		a1Sign = 0.0;
	end
	
	if dx > dx_a2
		a2Sign = -1.0;
	elseif dx < dx_a2
		a2Sign = 1.0;
	else
		a2Sign = 0.0;
	end
	
	% Solve for the three cases
	if a1Sign == 1.0 && a2Sign == -1.0 % ACC_DEC
		% Assume the move is a saturated approaching a peak profile only when dx = dx_max and v_p <= v_max
		solution.move = PATH_ACC_DEC_SATURATED;
		
		p_2 = - 1.0;
		p_1 = a * (dt - dt_bar) + 2.0 * max(v_0, v_f);
		p_0 = (-1.0) * max(v_0, v_f) ^ 2 - a * (dx - dx_bar);
		
	elseif a1Sign == a2Sign % ACC_ACC or DEC_DEC
		if a1Sign == 1.0
			solution.move = PATH_ACC_ACC;
			solution.v(2) = (dx - dx_bar) / (dt - dt_bar);
		elseif a1Sign == -1.0
			solution.move = PATH_DEC_DEC;
			solution.v(2) = (dx - dx_bar) / (dt - dt_bar);
		else
			if v_f > v_0
				solution.move = PATH_ACC_ACC;
			else
				solution.move = PATH_DEC_DEC;
			end
			solution.v(2) = v_f;
		end
		
	else % DEC_ACC
		solution.move = PATH_DEC_ACC_SATURATED;
		
		p_2 = 1.0;
		p_1 = a * (dt - dt_bar) - 2.0 * min(v_0, v_f);
		p_0 = min(v_0, v_f) ^ 2 - a * (dx - dx_bar);
		
	end
	
	% Use quadratic function
	if (solution.move == PATH_ACC_DEC_SATURATED) || (solution.move == PATH_DEC_ACC_SATURATED)
		[rootsSolution, rootsValid] = PathRoots(p_2, p_1, p_0);
		
		if !rootsValid
			printf("PathVel call failed: Invalid roots for movement\n");
			valid = false;
			return;
			
		else
			% Choose the appropriate root
			if solution.move == PATH_ACC_DEC_SATURATED
				solution.v(2) = min(rootsSolution.r_1, rootsSolution.r_2);
			else
				solution.v(2) = max(rootsSolution.r_1, rootsSolution.r_2);
			end
			
		end
	end
	
	solution.v(1) = v_0;
	solution.v(3) = solution.v(2);
	solution.v(4) = v_f;
	solution.t(2) = abs(v_0 - solution.v(2)) / a;
	solution.t(3) = dt - abs(solution.v(3) - v_f) / a;
	solution.t(4) = dt;
	solution.a = a;
	solution.dx = dx;
	valid = true;
	
	if printResult
		printf("PathVel call: Vel %.3f, Move %d\n", solution.v(2), solution.move);
	end
	
end
