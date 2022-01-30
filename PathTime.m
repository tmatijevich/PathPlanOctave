%!octave

% FUNCTION NAME:
%   PathTime
%
% DESCRIPTION:
%   Minimum time to move with acceleration over a distance
%
% INPUT:
%   dx    - Distance [Units]
%   v_0   - Initial velocity [Units/s]
%   v_f   - Final velocity [Units/s]
%   v_min - Minimum velocity [Units/s]
%   v_max - Maximum velocity [Units/s]
%   a     - Acceleration magnitude [Units/s^2]
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Base solution type
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
%   2020-12-23
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathTime(dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Reference global variables
	run GlobalVar;
	
	% Reset solution
	solution = struct("t_", [0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE);
	valid = false;
	
	% Input requirements
	% #1 Plausible velocity limits
	if v_min < 0.0 || v_max <= v_min
		printf("PathTime call failed: Implausible velocity limits [%.3f, %.3f] u/s\n", v_min, v_max); 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathTime call failed: Endpoint velocities %.3f, %.3f u/s exceed limits [%.3f, %.3f] u/s\n", v_0, v_f, v_min, v_max); 
		return;
	
	% #3 Positive inputs
	elseif dx <= 0.0 || a <= 0.0
		printf("PathTime call failed: Distance %.3f u or acceleration %.3f u/s^2 non-positive\n", dx, a); 
		return;
		
	% #4 Plausible move
	elseif dx < (abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a))
		printf("PathTime call failed: Distance %.3f u subceeds minimum %.3f u\n", dx, abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a)); 
		return;
	end
	
	% There is no time advantage to decelerating below the final velocity, therefore only Acc/Dec profiles will be considered
	dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
	
	% Saturated?
	if dx < dx_u % Peak
		solution.move = PATH_MOVE_ACCDECPEAK;
		
		% Compute peak velocity
		v_p = sqrt(dx * a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		
		% Set solution
		solution.t_(2) = (v_p - v_0) / a;
		solution.t_(3) = solution.t_(2);
		solution.t_(4) = solution.t_(3) + (v_p - v_f) / a;
		solution.v_(2) = v_p;
		solution.v_(3) = v_p;
		
	else % Saturated
		solution.move = PATH_MOVE_ACCDECSATURATED;
		
		% Compute time at v_max
		t_12 = (dx - dx_u) / v_max;
		
		% Set solution
		solution.t_(2) = (v_max - v_0) / a;
		solution.t_(3) = solution.t_(2) + t_12;
		solution.t_(4) = solution.t_(3) + (v_max - v_f) / a;
		solution.v_(2) = v_max;
		solution.v_(3) = v_max;
	end 
	
	% Set remaining solution
	solution.dx 	= dx;
	solution.v_(1) 	= v_0;
	solution.v_(4) 	= v_f;
	solution.a 		= a;
	valid = true;
	
	if printResult
		printf("PathTime call: Time %.3f s, Vel %.3f u/s, Move %s\n", solution.t_(4), solution.v_(2), GetMove(solution.move));
	end
	
end % Function definition
