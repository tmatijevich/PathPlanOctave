%!octave

% FUNCTION NAME: 
%   PathAcc
%
% DESCRIPTION: 
%   Minimum acceleration to move in time over a distance
%
% INPUT:
%   dt    - Time duration [s]
%   dx    - Distance [Units]
%   v_0   - Initial velocity [Units/s]
%   v_f   - Final velocity [Units/s]
%   v_min - Minimum velocity [Units/s]
%   v_max - Maximum velocity [Units/s]
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Base solution path
%     t_   - Time-point array [s]
%     dx   - Distance [Units]
%     v_   - Velocity-point array [Units/s]
%     a    - Acceleration magnitude [Units/s^2]
%     move - Movement type (global)
%   valid - Successful completion
%
% ASSUMPTIONS AND LIMITATIONS:
%   - Positive distance and velocity
%   - Symmetric acceleration and deceleration
%   - Zero jerk
%
% DATE CREATED: 
%   2020-04-10
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathAcc(dt, dx, v_0, v_f, v_min, v_max, printResult = false)
	% Reference global variables
	run GlobalVars;
	
	% Reset solution
	solution = struct('t_', [0.0, 0.0, 0.0, 0.0], 'dx', 0.0, 'v_', [0.0, 0.0, 0.0, 0.0], 'a', 0.0, 'move', PATH_MOVE_NONE);
	
	% Input requirements
	% #1 Plausible velocity limits
	if v_min < 0.0 || v_max <= v_min
		printf("PathAcc call failed: Implausible velocity limits %.3f, %.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathAcc call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v_0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3 Positive inputs
	elseif dt <= 0.0 || dx <= 0.0
		printf("PathAcc call failed: Time duration %.3f or distance %.3f non-positive\n", dt, dx); 
		valid = false; 
		return;
		
	% #4 Plausible move
	elseif dx <= (v_min * dt) || dx >= (v_max * dt)
		printf("PathAcc call failed: Impossible distance %.3f given limits %.3f, %.3f\n", dx, v_min * dt, v_max * dt); 
		valid = false; 
		return;
		
	end % Requirements
	
	% The intermediate velocity v_12 is >= v_0, v_f or <= v_0, v_f in a symmetric acc & dec profile
	
	% AccDec or DecAcc?
	dx_bar = 0.5 * dt * (v_0 + v_f); % Area of a trapezoid
	
	if dx >= dx_bar % AccDec
		% Saturated?
		dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * ((2.0 * v_max - v_0 - v_f) / dt));
		% NOTE: There is no dx >= dx_bar when v_0 = v_f = v_max that also passes requirement #4. This protects against divide by zero.
		
		if dx < dx_u % AccDec with peak
			solution.move = PATH_MOVE_ACCDECPEAK;
			
		else % AccDec saturated
			solution.move = PATH_MOVE_ACCDECSATURATED;
			solution.a = ((2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / 2.0 - (2.0 * v_max - v_0 - v_f) * v_max) / (dx - dt * v_max); % Protected by requirement #4
			if solution.a > 0.0
				solution.v_(2) = v_max;
				solution.v_(3) = v_max;
				solution.t_(2) = (v_max - v_0) / solution.a;
				solution.t_(3) = dt - (v_max - v_f) / solution.a;
			else
				printf("PathAcc call warning: Unexpected non-positive acceleration\n"); % Handle any floating point inaccuracy when passing requirement #4
				solution.v_(2) = v_0;
				solution.v_(3) = v_0;
				solution.t_(2) = 0.0;
				solution.t_(3) = 0.0;
			end % Positive acceleration
			
		end % dx_u?
		
	else % DecAcc
		% Saturated?
		dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * ((v_0 + v_f - 2.0 * v_min) / dt));
		% NOTE: There is no dx < dx_bar when v_0 = v_f = v_min that also passes requirement #4. This protects against divide by zero.
		
		if dx > dx_l % DecAcc with peak (dip)
			solution.move = PATH_MOVE_DECACCPEAK;
			
		else % DecAcc saturated
			solution.move = PATH_MOVE_DECACCSATURATED;
			solution.a = ((v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / 2.0 - (v_0 + v_f - 2.0 * v_min) * v_min) / (dx - dt * v_min); % Protected by requirement #4
			if solution.a > 0.0
				solution.v_(2) = v_min;
				solution.v_(3) = v_min;
				solution.t_(2) = (v_0 - v_min) / solution.a;
				solution.t_(3) = dt - (v_f - v_min) / solution.a;
			else 
				printf("PathAcc call warning: Unexpected non-positive acceleration\n"); % Handle any floating point inaccuracy when passing requirement #4
				solution.v_(2) = v_0;
				solution.v_(3) = v_0;
				solution.t_(2) = 0.0;
				solution.t_(3) = 0.0;
			end % Positive acceleration
		end % dx_l?
		
	end % dx_bar?
	
	if (solution.move == PATH_MOVE_ACCDECPEAK) || (solution.move == PATH_MOVE_DECACCPEAK)
		p_2 = 2.0 * dt;
		p_1 = -4.0 * dx;
		p_0 = 2.0 * dx * (v_0 + v_f) - dt * (v_0 ^ 2 + v_f ^ 2);
		[rootsSolution, rootsValid] = PathRoots(p_2, p_1, p_0);
		
		if !rootsValid
			printf("PathAcc call failed: Invalid roots for peak movement\n");
			valid = false;
			return;
			
		else % Roots are valid
			% Peak velocity
			if solution.move == PATH_MOVE_ACCDECPEAK 
				solution.v_(2) = max(rootsSolution.r_1, rootsSolution.r_2);
				solution.v_(3) = solution.v_(2);
			else
				solution.v_(2) = min(rootsSolution.r_1, rootsSolution.r_2);
				solution.v_(3) = solution.v_(2);
			end
			
			% Acceleration magnitude
			solution.a = abs(2.0 * solution.v_(2) - v_0 - v_f) / dt;
			if solution.a > 0.0
				solution.t_(2) = abs(solution.v_(2) - v_0) / solution.a;
				solution.t_(3) = solution.t_(2);
			else % A flat line, dx = dx_bar and v_0 = v_f
				solution.t_(2) = 0.0;
				solution.t_(3) = 0.0;
			end
			
		end % Roots valid?
	end % Peak movement?
	
	% Finish solution
	solution.t_(4) = dt;
	solution.dx = dx;
	solution.v_(1) = v_0;
	solution.v_(4) = v_f;
	valid = true;
	
	if printResult
		printf("PathAcc call: Acc %.3f, Vel %.3f, Move %d\n", solution.a, solution.v_(2), solution.move);
	end
	
end % Function
