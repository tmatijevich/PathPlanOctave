%!octave

% FUNCTION NAME: 
%   PathDist
%
% DESCRIPTION: 
%   Maximum distance from move with acceleration in time
%
% INPUT:
%   dt    - Time duration [s]
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
%   2020-12-29
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathDist(dt, v_0, v_f, v_min, v_max, a, printResult = false)
	% Reference global variables
	run GlobalVar;
	
	% Reset solution
	solution = struct("t_", [0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE);
	valid = false;
	
	% Input requirements
	% #1 Plausible velocity limits
	if v_min < 0.0 || v_max <= v_min
		printf("PathDist call failed: Implausible velocity limits [%.3f, %.3f] u/s\n", v_min, v_max); 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathDist call failed: Endpoint velocities %.3f, %.3f u/s exceed limits [%.3f, %.3f] u/s\n", v_0, v_f, v_min, v_max); 
		return;
	
	% #3 Positive inputs
	elseif dt <= 0.0 || a <= 0.0
		printf("PathDist call failed: Duration %.3f s or acceleration %.3f u/s^2 non-positive\n", dt, a); 
		return;
		
	% #4 Plausible move
	elseif dt < (abs(v_0 - v_f) / a)
		printf("PathDist call failed: Duration %.3f s subceeds minimum %.3f s\n", dt, (abs(v_0 - v_f) / a)); 
		return;
	end
	
	% There is no advantage from decelerating then accelerating during move, only Acc/Dec profiles will be considered
	dt_u = (2.0 * v_max - v_0 - v_f) / a;
	
	% Saturated?
	if dt < dt_u % Peak
		solution.move = PATH_MOVE_ACCDECPEAK;
		
		% Compute peak velocity
		v_p = (dt * a + v_0 + v_f) / 2.0;
		
		% Set solution
		solution.dx = (2.0 * v_p ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
		solution.t(2) = (v_p - v_0) / a;
		solution.t(3) = solution.t(2);
		solution.v(2) = v_p;
		solution.v(3) = v_p;
		
	else % Saturated
		solution.move = PATH_MOVE_ACCDECSATURATED;
		
		% Compute distance at v_max
		dx_12 = (dt - (2.0 * v_max - v_0 - v_f) / a) * v_max;
		
		% Set solution
		solution.dx = dx_12 + (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a);
		solution.t(2) = (v_max - v_0) / a;
		solution.t(3) = solution.t(2) + dx_12 / v_max;
		solution.v(2) = v_max;
		solution.v(3) = v_max;
	end
	
	% Set remaining solution
	solution.t(4) 	= dt;
	solution.v(1) 	= v_0;
	solution.v(4) 	= v_f;
	solution.a 		= a;
	valid = true;
	
	if printResult
		printf("PathDist call: Dist %.3f u, Vel %.3f u/s Move %s\n", solution.dx, solution.v(2), GetMove(solution.move));
	end
	
end % Function definition
