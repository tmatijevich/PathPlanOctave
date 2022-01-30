%!octave

% FUNCTION NAME: 
%   PathTimeDiff
%
% DESCRIPTION: 
%   Difference in time durations between fastest and slowest moves
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
%   solution (struct) - Time difference solution path
%     accDec (struct)
%       t_   - Time-point array [s]
%       dx   - Distance [Units]
%       v_   - Velocity-point array [Units/s]
%       a    - Acceleration magnitude [Units/s^2]
%       move - Movement type
%     decAcc
%       (same as accDec)
%     dt_tilde - Difference in duration between accDec and decAcc profiles [s]
%   valid - Successful completion
%
% ASSUMPTIONS AND LIMITATIONS:
%   - Positive distance and velocity
%   - Symmetric acceleration and deceleration
%   - Zero jerk
%
% DATE CREATED: 
%   2020-03-25
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathTimeDiff(dx, v_0, v_f, v_min, v_max, a, printResult = false)
	% Reference global variables
	run GlobalVar;
	
	% Reset solution
	solution = struct("accDec", struct("t_", [0.0, 0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE),
				"decAcc", struct("t_", [0.0, 0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE),
				"dt_tilde", 0.0);
	valid = false;
	
	% Input requirements
	% #1 Plausible velocity limits
	if v_min < 0.0 || v_max <= v_min
		printf("PathTimeDiff call failed: Implausible velocity limits [%.3f, %.3f] u/s\n", v_min, v_max); 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathTimeDiff call failed: Endpoint velocities %.3f, %.3f u/s exceed limits [%.3f, %.3f] u/s\n", v_0, v_f, v_min, v_max); 
		return;
	
	% #3 Positive inputs
	elseif dx <= 0.0 || a <= 0.0
		printf("PathTimeDiff call failed: Distance %.3f u or acceleration %.3f u/s^2 non-positive\n", dx, a); 
		return;
		
	% #4 Plausible move
	elseif dx < (abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a))
		printf("PathTimeDiff call failed: Distance %.3f u subceeds minimum %.3f u\n", dx, abs(v_0 ^ 2 - v_f ^ 2) / (2.0 * a)); 
		return;
	end
	
	% Time-minimizing profile
	dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a); % Distance at maximum saturation limit
	if dx < dx_u % Peak
		solution.accDec.move = PATH_MOVE_ACCDECPEAK;
		
		% Compute v_12
		solution.accDec.v_(2) = sqrt(dx * a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		solution.accDec.v_(3) = solution.accDec.v_(2);
		solution.accDec.t_(2) = (solution.accDec.v_(2) - v_0) / a;
		solution.accDec.t_(3) = solution.accDec.t_(2);
		solution.accDec.t_(4) = solution.accDec.t_(3) + (solution.accDec.v_(3) - v_f) / a;
		
	else % Saturated
		solution.accDec.move = PATH_MOVE_ACCDECSATURATED;
		
		% Compute time at v_max
		dt_12 = (dx - dx_u) / v_max;
		solution.accDec.v_(3) = v_max;
		solution.accDec.v_(2) = v_max;
		solution.accDec.t_(2) = (v_max - v_0) / a;
		solution.accDec.t_(3) = solution.accDec.t_(2) + dt_12;
		solution.accDec.t_(4) = solution.accDec.t_(3) + (v_max - v_f) / a;
	end 
	
	% Time-maximizing profile
	dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a);
	if dx < dx_l % Peak (dip)
		solution.decAcc.move = PATH_MOVE_DECACCPEAK;
		
		% Compute v_12
		solution.decAcc.v_(2) = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * a);
		solution.decAcc.v_(3) = solution.decAcc.v_(2);
		solution.decAcc.t_(2) = (v_0 - solution.decAcc.v_(2)) / a;
		solution.decAcc.t_(3) = solution.decAcc.t_(2);
		solution.decAcc.t_(4) = solution.decAcc.t_(3) + (v_f - solution.decAcc.v_(3)) / a;
		
	else % Saturated
		solution.decAcc.move = PATH_DEC_ACC_SATURATED;
		
		% Compute time at v_min
		dt_12 = (dx - dx_l) / v_min;
		solution.decAcc.v_(2) = v_min;
		solution.decAcc.v_(3) = v_min;
		solution.decAcc.t_(2) = (v_0 - v_min) / a;
		solution.decAcc.t_(3) = solution.decAcc.t_(2) + dt_12;
		solution.decAcc.t_(4) = solution.decAcc.t_(3) + (v_f - v_min) / a;
	end 
	
	% Set solution
	solution.accDec.v_(1) = v_0;
	solution.accDec.v_(4) = v_f;
	solution.decAcc.v_(1) = v_0;
	solution.decAcc.v_(4) = v_f;
	solution.dt_tilde = solution.decAcc.t_(4) - solution.accDec.t_(4);
	valid = true;
	
	if printResult
		printf("PathTimeDiff call: Time diff %.3f - %.3f = %.3f s, Moves %s, %s\n", solution.decAcc.t_(4), solution.accDec.t_(4), solution.dt_tilde, GetMove(solution.accDec.move), GetMove(solution.decAcc.move));
	end
	
end % Function definition
