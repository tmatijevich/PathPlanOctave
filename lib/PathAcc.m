%!octave

function [solution, valid] = PathAcc(dt, dx, v_0, v_f, v_min, v_max, printResult = false)
	% PathAcc(dt, dx, v_0, v_f, v_min, v_max, printResult = false)
	% Determine the minimum acceleration to change velocity in time over a distance
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-04-10
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	solution.t 		= [0.0, 0.0, 0.0, 0.0];
	solution.dx 	= 0.0;
	solution.v 		= [0.0, 0.0, 0.0, 0.0];
	solution.a 		= 0.0;
	solution.move 	= PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (v_min < 0.0) || (v_max <= v_min)
		printf("PathAcc call failed: Implausible velocity limits %.3f, %.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < v_min) || (v_0 > v_max) || (v_f < v_min) || (v_f > v_max)
		printf("PathAcc call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive time duration and distance
	elseif (dt <= 0.0) || (dx <= 0.0)
		printf("PathAcc call failed: Time or distance non-positive %.3f, %.3f\n", dt, dx); 
		valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (dx <= (v_min * dt)) || (dx >= (v_max * dt))
		printf("PathAcc call failed: Impossible distance %.3f given limits %.3f, %.3f\n", dx, v_min * dt, v_max * dt); 
		valid = false; 
		return;
		
	end % Requirements
	
	% The intermediate velocity point v12 is either >= v_0, v_f or <= v_0, v_f for a symmetric acc/dec profile
	% Determine if 1. ACC 2. DEC or 1. DEC 2. ACC
	dx_bar = 0.5 * dt * (v_0 + v_f); % Area of a trapezoid
	
	if dx >= dx_bar % 1. ACC 2. DEC
		% Determine if saturated
		dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * ((2.0 * v_max - v_0 - v_f) / dt));
		% NOTE: There is no dx >= dx_bar when v_0 = v_f = v_max that also passes requirement #4. This protects against divide by zero.
		
		if dx < dx_u % Acc/dec profile with peak
			solution.move = PATH_ACC_DEC_PEAK;
			
		else % Acc/dec profile saturated at v_max
			solution.move = PATH_ACC_DEC_SATURATED;
			solution.a = ((2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / 2.0 - (2.0 * v_max - v_0 - v_f) * v_max) / (dx - dt * v_max); % Protected by requirement #4
			if solution.a > 0.0
				solution.v(2) = v_max;
				solution.v(3) = v_max;
				solution.t(2) = (v_max - v_0) / solution.a;
				solution.t(3) = dt - (v_max - v_f) / solution.a;
			else
				printf("PathAcc call warning: Unexpected non-positive acceleration\n"); % Should not happen
				solution.v(2) = v_0;
				solution.v(3) = v_0;
				solution.t(2) = 0.0;
				solution.t(3) = 0.0;
			end % Positive acceleration
			
		end % dx_u?
		
	else % 1. DEC 2. ACC
		% Determine if saturated profile
		dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * ((v_0 + v_f - 2.0 * v_min) / dt));
		% NOTE: There is no dx < dx_bar when v_0 = v_f = v_min that also passes requirement #4. This protects against divide by zero.
		
		if dx > dx_l % Dec/acc profile with dip
			solution.move = PATH_DEC_ACC_PEAK;
			
		else % Dec/acc profile saturated at v_min
			solution.move = PATH_DEC_ACC_SATURATED;
			solution.a = ((v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / 2.0 - (v_0 + v_f - 2.0 * v_min) * v_min) / (dx - dt * v_min); % Protected by requirement #4
			if solution.a > 0.0
				solution.v(2) = v_min;
				solution.v(3) = v_min;
				solution.t(2) = (v_0 - v_min) / solution.a;
				solution.t(3) = dt - (v_f - v_min) / solution.a;
			else 
				printf("PathAcc call warning: Unexpected non-positive acceleration\n"); % Should not happen
				solution.v(2) = v_0;
				solution.v(3) = v_0;
				solution.t(2) = 0.0;
				solution.t(3) = 0.0;
			end % Positive acceleration
		end % dx_l?
		
	end % dx_bar?
	
	if (solution.move == PATH_ACC_DEC_PEAK) || (solution.move == PATH_DEC_ACC_PEAK)
		p_2 = 2.0 * dt;
		p_1 = -4.0 * dx;
		p_0 = 2.0 * dx * (v_0 + v_f) - dt * (v_0 ^ 2 + v_f ^ 2);
		[rootsSolution, rootsValid] = PathRoots(p_2, p_1, p_0);
		
		if !rootsValid
			printf("PathAcc call failed: Invalid roots for peak movement\n");
			valid = false;
			return;
			
		else % Roots are valid
			if solution.move == PATH_ACC_DEC_PEAK % Vmax
				solution.v(2) = max(rootsSolution.r_1, rootsSolution.r_2);
				solution.v(3) = solution.v(2);
				
			else % Vmin
				solution.v(2) = min(rootsSolution.r_1, rootsSolution.r_2);
				solution.v(3) = solution.v(2);
				
			end
			
			solution.a = abs(2.0 * solution.v(2) - v_0 - v_f) / dt;
			if solution.a > 0.0
				solution.t(2) = abs(solution.v(2) - v_0) / solution.a;
				solution.t(3) = solution.t(2);
			else % A flat line, dx = dx_bar and v_0 = v_f
				solution.t(2) = 0.0;
				solution.t(3) = 0.0;
			end
			
		end % Roots valid?
	end % Peak movement?
	
	% Set common solution values and validate
	solution.t(4) = dt;
	solution.dx = dx;
	solution.v(1) = v_0;
	solution.v(4) = v_f;
	valid = true;
	
	if printResult
		printf("PathAcc call: Acc %.3f, Vel %.3f, Move %d\n", solution.a, solution.v(2), solution.move);
	end
	
end % Function
