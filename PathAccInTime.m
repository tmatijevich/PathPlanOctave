%!octave

% FUNCTION NAME:
%   PathAccInTime
%
% INPUT:
%   dt_tilde - Difference in time durations [s]
%   dx       - Distance [Units]
%   v_0      - Initiali velocity [Units/s]
%   v_f      - Final velocity [Units/s]
%   v_min    - Minimum velocity [Units/s]
%   v_max    - Maximum velocity [Units/s]
%   printResult - Print successful completion message
%
% OUTPUT:
%   solution (struct) - Time-difference solution
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
%   2020-04-01
%
% AUTHOR:
%   Tyler Matijevich
%

function [solution, valid] = PathAccInTime(dt_tilde, dx, v_0, v_f, v_min, v_max, printResult = false)
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
		printf("PathAccInTime call failed: Implausible velocity limits [%.3f, %.3f]\n", v_min, v_max); 
		return;
	
	% #2 Valid endpoint velocities
	elseif v_0 < v_min || v_max < v_0 || v_f < v_min || v_max < v_f
		printf("PathAccInTime call failed: Endpoint velocities %.3f, %.3f exceed limits [%.3f, %.3f]\n", v_0, v_f, v_min, v_max); 
		return;
	
	% #3 Positive inputs
	elseif dt_tilde <= 0.0 || dx <= 0.0
		printf("PathAccInTime call failed: Time difference %.3f s or distance %.3f non-positive\n", dt_tilde, dx); 
		return;
		
	% #4 Plausible move
	elseif dt_tilde >= dx / v_min - dx / v_max 
		printf("PathAccInTime call failed: Time difference %.3f s exceeds maximum %.3f s\n", dt_tilde, dx / v_min - dx / v_max); 
		return;
	end

	% Saturated acceleration limits
	a_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * dx); % AccDec, a > a_u causes saturation
	a_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * dx); % DecAcc

	% Saturated duration limits
	dt_u = (2.0 * v_max - v_0 - v_f) / a_u;
	dt_l = (v_0 + v_f - 2.0 * v_min) / a_l;

	% Time durations at alternative path saturation limit
	if a_u > a_l
		% AccDec path is not saturated at a_l
		v_12 = sqrt(dx * a_l + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		dt_u_hat = (2.0 * v_12 - v_0 - v_f) / a_l;

		% DecAcc path is saturated at a_u
		dt_12 = (dx - (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a_u)) / v_min;
		dt_l_hat = (v_0 + v_f - 2.0 * v_min) / a_u + dt_12;
	else
		% AccDec path is saturated at a_l
		dt_12 = (dx - (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * a_l)) / v_max;
		dt_u_hat = (2.0 * v_max - v_0 - v_f) / a_l + dt_12;

		% DecAcc path is not saturated at a_u
		v_12 = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * a_u);
		dt_l_hat = (v_0 + v_f - 2.0 * v_12) / a_u;
	end

	% Saturated time difference limits
	dt_u_tilde = dt_l_hat - dt_u; % dt_l_hat >= dt_u for a_u
	dt_l_tilde = dt_l - dt_u_hat; % dt_l >= dt_u_hat for a_l

	% Computation reduction constants
	c_dt_u = 2.0 * v_max - v_0 - v_f;
	c_dx_u = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * v_max);
	c_dt_l = v_0 + v_f - 2.0 * v_min;
	c_dx_l = (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * v_min);

	% Four cases
	% 1. dt_tilde >= dt_u_tilde && dt_tilde >= dt_l_tilde
	% 2. dt_tilde  < dt_u_tilde && dt_tilde >= dt_l_tilde
	% 3. dt_tilde >= dt_u_tilde && dt_tilde  < dt_l_tilde
	% 4. dt_tilde  < dt_u_tilde && dt_tilde  < dt_l_tilde

	flagRoots = false;

	% 1
	if dt_tilde >= dt_u_tilde && dt_tilde >= dt_l_tilde
		solution.accDec.move = PATH_MOVE_ACCDECSATURATED;
		solution.decAcc.move = PATH_MOVE_DECACCSATURATED;

		solution.accDec.a = (c_dt_l - c_dt_u - (c_dx_l - c_dt_u)) / (dt_tilde - dx * (1.0 / v_min - 1.0 / v_max));
		solution.decAcc.a = solution.accDec.a;

	% 2
	elseif dt_tilde < dt_u_tilde && dt_tilde >= dt_l_tilde
		solution.accDec.move = PATH_MOVE_ACCDECPEAK;
		solution.decAcc.move = PATH_MOVE_DECACCSATURATED;

		c_1 = (v_0 ^ 2 + v_f ^ 2) / 2.0;
		c_2 = c_dt_l - c_dx_l + v_0 + v_f;
		c_3 = dt_tilde - (dx / v_min);
		p_2 = c_3 ^ 2;
		p_1 = -4.0 * dx - 2.0 * c_2 * c_3;
		p_0 = c_2 ^ 2 - 4.0 * c_1;

		flagRoots = true;

	% 3
	elseif dt_tilde >= dt_u_tilde && dt_tilde < dt_l_tilde
		solution.accDec.move = PATH_MOVE_ACCDECSATURATED;
		solution.decAcc.move = PATH_MOVE_DECACCPEAK;

		c_1 = (v_0 ^ 2 + v_f ^ 2) / 2.0;
		c_2 = -c_dt_u + c_dx_u + v_0 + v_f;
		c_3 = dt_tilde + (dx / v_min);
		p_2 = c_3 ^ 2;
		p_1 = 4.0 * dx - 2.0 * c_2 * c_3;
		p_0 = c_2 ^ 2 - 4.0 * c_1;

		flagRoots = true;

	%4 
	else
		if dt_u_tilde > dt_l_tilde
			solution.accDec.move = PATH_MOVE_ACCDECPEAK;
			solution.decAcc.move = PATH_MOVE_DECACCSATURATED;
			
			solution.accDec.a = a_l;
			solution.decAcc.a = a_l;
		else
			solution.accDec.move = PATH_MOVE_ACCDECSATURATED;
			solution.decAcc.move = PATH_MOVE_DECACCPEAK;
			
			solution.accDec.a = a_u;
			solution.decAcc.a = a_u;
		end
	end

	% Compute roots for cases 2 & 3
	if flagRoots
		[rootsValid, rootsSolution] = PathRoots(p_2, p_1, p_0);

		if !rootsValid
			printf("PathAccInTime call failed: Unable to find roots\n");
			return;
		end

		solution.accDec.a = max(rootsSolution.r_1, rootsSolution.r_2);
		solution.decAcc.a = solution.accDec.a;
	end

	% Set solution
	solution.accDec.dx = dx;
	if solution.accDec.move == PATH_MOVE_ACCDECPEAK
		v_12 = sqrt(dx * solution.accDec.a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		solution.accDec.v_(1) = v_0;
		solution.accDec.v_(2) = v_12;
		solution.accDec.v_(3) = v_12;
		solution.accDec.v_(4) = v_f;
		solution.accDec.t_(2) = (v_12 - v_0) / solution.accDec.a;
		solution.accDec.t_(3) = solution.accDec.t_(2);
		solution.accDec.t_(4) = solution.accDec.t_(3) + (v_12 - v_f) / solution.accDec.a;
	else
		solution.accDec.v_(1) = v_0;
		solution.accDec.v_(2) = v_max;
		solution.accDec.v_(3) = v_max;
		solution.accDec.v_(4) = v_f;
		solution.accDec.t_(2) = (v_max - v_0) / solution.accDec.a;
		solution.accDec.t_(3) = solution.accDec.t_(2) + (dx - (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * solution.accDec.a)) / v_max;
		solution.accDec.t_(4) = solution.accDec.t_(3) + (v_max - v_f) / solution.accDec.a;
	end

	solution.decAcc.dx = dx;
	if solution.decAcc.move == PATH_MOVE_DECACCPEAK
		v_12 = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * solution.decAcc.a);
		solution.decAcc.v_(1) = v_0;
		solution.decAcc.v_(2) = v_12;
		solution.decAcc.v_(3) = v_12;
		solution.decAcc.v_(4) = v_f;
		solution.decAcc.t_(2) = (v_0 - v_12) / solution.decAcc.a;
		solution.decAcc.t_(3) = solution.decAcc.t_(2);
		solution.decAcc.t_(4) = solution.decAcc.t_(3) + (v_f - v_12) / solution.decAcc.a;
	else
		solution.decAcc.v_(1) = v_0;
		solution.decAcc.v_(2) = v_min;
		solution.decAcc.v_(3) = v_min;
		solution.decAcc.v_(4) = v_f;
		solution.decAcc.t_(2) = (v_0 - v_min) / solution.decAcc.a;
		solution.decAcc.t_(3) = solution.decAcc.t_(2) + (dx - (v_0 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * solution.decAcc.a)) / v_max;
		solution.decAcc.t_(4) = solution.decAcc.t_(3) + (v_f - v_min) / solution.decAcc.a;
	end

	valid = true;

	if printResult
		printf("PathAccInTime call: Acc %.3f u/s^2 Move %s & %s\n", solution.accDec.a, GetMove(solution.accDec.move), GetMove(solution.decAcc.move));
	end

end % Function