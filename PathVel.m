%!octave

% FUNCTION NAME: 
%   PathVel
%
% DESCRIPTION: 
%   Minimum velocity to move with acceleration in time over a distance
%
% INPUT:
%   dt    - Time duration [s]
%   dx    - Distance [Units]
%   v_0   - Initial velocity [Units/s]
%   v_f   - Final velocity [Units/s]
%   v_min - Minimum velocity [Units/s]
%   v_max - Maximum velocity [Units/s]
%   a     - Acceleration magnitude [Units/s^2]
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Base solution path
%     t_   - Time-point array [s]
%     dx   - Distance [Units]
%     v_   - Velocity-point array [Units/s]
%     a    - Acceleration magnitude [Units/s^2]
%     move - Movement type
%   valid - Successful completion
%
% ASSUMPTIONS AND LIMITATIONS:
%   - Positive distance and velocity
%   - Symmetric acceleration and deceleration
%   - Zero jerk
%
% DATE CREATED: 
%   2020-12-30
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathVel(dt, dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Reference global variables
	run GlobalVar;
	
	% Reset solution
	solution = struct("t_", [0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE);
	valid = false;
	
	% Input requirements
	% #1 Plausible velocity limits
	if v_min < 0.0 || v_max <= v_min
		printf("PathVel call failed: Implausible velocity limits [%.3f, %.3f] u/s\n", v_min, v_max); 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathVel call failed: Endpoint velocities %.3f, %.3f u/s exceed limits [%.3f, %.3f] u/s\n", v_0, v_f, v_min, v_max); 
		return;
	
	% #3 Positive inputs
	elseif dt <= 0.0 || dx <= 0.0 || a <= 0.0 
		printf("PathVel call failed: Duration %.3f s, distance %.3f u, or acceleration %.3f u/s^2 non-positive\n", dt, dx, a); 
		return;
		
	% #4 Plausible move
	elseif dt < abs(v_0 - v_f) / a
		printf("PathVel call failed: Duration %.3f s subceeds minimum %.3f s\n", dt, abs(v_0 - v_f) / a); 
		return;
	end
	
	% #4.1 Plausible move
	% Check distance (given acceleration and time)
	v_12 = (a * dt + v_0 + v_f) / 2.0; % Test maximum v_12
	if v_12 <= v_max % Peak
		dx_max = (2.0 * v_12 ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	else % Saturated
		dx_max = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a) + v_max * (dt - ((2.0 * v_max - v_0 - v_f) / a));
	end

	v_12 = (v_0 + v_f - a * dt) / 2.0; % Test minimum v_12
	if v_12 >= v_min % Dip
		dx_min = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_12 ^ 2) / (2.0 * a);
	else % Saturated
		dx_min = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a) + v_min * (dt - ((v_0 + v_f - 2.0 * v_min) / a));
	end

	if dx < dx_min || dx > dx_max
		printf("PathVel call failed: Distance %.3f u exceeds limits [%.3f, %.3f] u\n", dx, dx_min, dx_max);
		return;
	end
	
	% Compute nominal distance and time
	dt_bar = abs(v_0 - v_f) / a;
	dx_bar = abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	dx_a1 = v_0 * (dt - dt_bar) + dx_bar; % Hold v_0 then acceleration to v_f
	dx_a2 = dx_bar + v_f * (dt - dt_bar); % Accelerate to v_f then hold v_f
	
	% Compute sign of acceleration phases
	sign_a1 = (dx > dx_a1) - (dx < dx_a1);
	sign_a2 = (dx < dx_a2) - (dx > dx_a2);
	
	% Three cases
	if sign_a1 == sign_a2 % Acc/Acc or Dec/Dec
		if sign_a1 >= 0.0
			solution.move = PATH_MOVE_ACCACC;
		else
			solution.move = PATH_MOVE_DECDEC;
		end
		if sign_a1 == 0.0
			solution.v_(2) = v_f; % dx == dx_a1 == dx_a2 and dt == dt_bar
		else
			solution.v_(2) = (dx - dx_bar) / (dt - dt_bar); 
		end

	elseif sign_a1 == 1.0 && sign_a2 == -1.0 % Acc/Dec
		solution.move = PATH_MOVE_ACCDECSATURATED; % Unsaturated only when dx == dx_max and v_12 < v_max
		
		p_2 = - 1.0;
		p_1 = a * (dt - dt_bar) + 2.0 * max(v_0, v_f);
		p_0 = (-1.0) * max(v_0, v_f) ^ 2 - a * (dx - dx_bar);
		
	else % Dec/Acc
		solution.move = PATH_MOVE_DECACCSATURATED;
		
		p_2 = 1.0;
		p_1 = a * (dt - dt_bar) - 2.0 * min(v_0, v_f);
		p_0 = min(v_0, v_f) ^ 2 - a * (dx - dx_bar);
		
	end
	
	% Find roots
	if solution.move == PATH_MOVE_ACCDECSATURATED || solution.move == PATH_MOVE_DECACCSATURATED
		[rootsSolution, rootsValid] = PathRoots(p_2, p_1, p_0);
		
		if !rootsValid
			printf("PathVel call failed: Invalid roots solution for move %s\n", GetMove(solution.move));
			return;
		end
		
		if solution.move == PATH_MOVE_ACCDECSATURATED
			solution.v_(2) = min(rootsSolution.r_1, rootsSolution.r_2);
		else
			solution.v_(2) = max(rootsSolution.r_1, rootsSolution.r_2);
		end
	end
	
	% Set remaining solution
	solution.dx 	= dx;
	solution.v_(1) 	= v_0;
	solution.v_(3) 	= solution.v_(2);
	solution.v_(4)	= v_f;
	solution.t_(2) 	= abs(v_0 - solution.v_(2)) / a;
	solution.t_(3) 	= max(dt - abs(solution.v_(3) - v_f) / a, solution.t_(2); % Ensure non-decreasing time given floating point subtraction
	solution.t_(4) 	= dt;
	solution.a 		= a;
	valid = true;
	
	if printResult
		printf("PathVel call: Vel %.3f u/s, Move %s\n", solution.v_(2), GetMove(solution.move));
	end
	
end % Function defintion
