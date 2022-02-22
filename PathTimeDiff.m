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
	run PathVar;
	
	% Reset solution
	solution = struct("accDec", struct("t_", [0.0, 0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE),
				"decAcc", struct("t_", [0.0, 0.0, 0.0, 0.0, 0.0], "dx", 0.0, "v_", [0.0, 0.0, 0.0, 0.0, 0.0], "a", 0.0, "move", PATH_MOVE_NONE),
				"dt_tilde", 0.0);
	valid = false;
	
	% Call PathTime for fastest path, checks inputs
	[timeSolution, timeValid] = PathTime(dx, v_0, v_f, v_min, v_max, a, false);
	if !timeValid 
		return;

	% Copy time-minimizing profile
	solution.accDec.move = timeSolution.move;
	solution.accDec.v(2) = timeSolution.v(2);
	solution.accDec.v(3) = timeSolution.v(3);
	solution.accDec.t(2) = timeSolution.t(2);
	solution.accDec.t(3) = timeSolution.t(3);
	solution.accDec.t(4) = timeSolution.t(4);
	
	% Time-maximizing profile
	dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a);
	% Saturated?
	if dx < dx_l % Peak (dip)
		solution.decAcc.move = PATH_MOVE_DECACC;
		
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
	
	% Set remaining solution
	solution.accDec.dx 		= dx;
	solution.accDec.v_(1) 	= v_0;
	solution.accDec.v_(4) 	= v_f;
	solution.accDec.a 		= a;
	solution.decAcc.dx 		= dx;
	solution.decAcc.v_(1) 	= v_0;
	solution.decAcc.v_(4) 	= v_f;
	solution.decAcc.a 		= a;
	solution.dt_tilde = solution.decAcc.t_(4) - solution.accDec.t_(4);
	valid = true;
	
	if printResult
		printf("PathTimeDiff call: Time diff %.3f - %.3f = %.3f s, Moves %s, %s\n", solution.decAcc.t_(4), solution.accDec.t_(4), solution.dt_tilde, PathMove(solution.accDec.move), PathMove(solution.decAcc.move));
	end
	
end % Function definition
