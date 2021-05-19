%!octave

function [solution, valid] = PathAccInTimeDiffWithRise(dt_tilde, dx, v_1, v_f, v_min, v_max, printResult = false)
	% PathAccInTimeDiffWithRise(dt_tilde, dx, v_1, v_f, v_min, v_max, printResult = false)
	% Determine the minimum acceleration to achieve movement extremes given the time difference and including a rise from standstill
	% Assumptions:
	% 	- Positive distance and velocity
	% 	- Symmetric acceleration and deceleration
	% 	- Zero jerk
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reference global variables
	global PATH_MOVE_NONE;
	global PATH_DEC_ACC_PEAK;
	global PATH_DEC_ACC_SATURATED;
	global PATH_ACC_DEC_PEAK;
	global PATH_ACC_DEC_SATURATED;
	
	% Reset solution
	solution.accDec.t 		= [0.0, 0.0, 0.0, 0.0, 0.0];
	solution.accDec.dx 		= 0.0;
	solution.accDec.v 		= [0.0, 0.0, 0.0, 0.0, 0.0];
	solution.accDec.a 		= 0.0;
	solution.accDec.move 	= PATH_MOVE_NONE;
	solution.decAcc.t 		= [0.0, 0.0, 0.0, 0.0, 0.0];
	solution.decAcc.dx 		= 0.0;
	solution.decAcc.v 		= [0.0, 0.0, 0.0, 0.0, 0.0];
	solution.decAcc.a 		= 0.0;
	solution.decAcc.move 	= PATH_MOVE_NONE;
	solution.case 			= 0;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (v_min < 0.0) || (v_max <= v_min)
		printf("PathAccInTimeDiffWithRise call failed: Implausible velocity limits %.3f, %.3f\n", v_min, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_1 < v_min) || (v_1 > v_max) || (v_f < v_min) || (v_f > v_max)
		printf("PathAccInTimeDiffWithRise call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v0, v_f, v_min, v_max); 
		valid = false; 
		return;
	
	% #3: Positive time difference and distance
	elseif (dt_tilde <= 0.0) || (dx <= 0.0)
		printf("PathAccInTimeDiffWithRise call failed: Time difference %.3f or distance %.3f is non-positive\n", dt_tilde, dx); 
		valid = false; 
		return;
		
	% #4: Valid time difference given distance and velocity limits
	elseif (dt_tilde >= (dx / v_min - dx / v_max))
		printf("PathAccInTimeDiffWithRise call failed: Time difference %.3f exceeds acceleration limit %.3f\n", dt_tilde, dx / v_min - dx / v_max); 
		valid = false; 
		return;
		
	end % Requirements
	
	% Determine the saturated acceleration limit for each profile
	a_u = (2.0 * v_max ^ 2 - v_f ^ 2) / (2.0 * dx); % Acceleration values exceeding this will saturated the Acc>Dec velocity profile
	a_l = (2.0 * v_1 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * dx);
	
	% Determine the saturated time duration limit for each profile
	dt_u = (2.0 * v_max - v_f) / a_u; % Time duration values subceeding this limit will saturate the Acc>Dec velocity profile
	dt_l = (2.0 * v_1 + v_f - 2.0 * v_min) / a_l;
	
	% Determine the time duration of each profile at the saturated acceleration limit of the alternate profile
	if a_u > a_l 
		% Acc>dec profile is not saturated at VminSatLimit
		v_p = sqrt(dx * a_l + (v_f ^ 2) / 2.0);
		dt_l_hat = (2.0 * v_p - v_f) / a_l;
		
		% Dec>acc profile saturated at VmaxSatLimit
		dt_23 = (dx - (2.0 * v_1 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * a_u)) / v_min;
		dt_u_hat = (2.0 * v_1 + v_f - 2.0 * v_min) / a_u + dt_23;
	else 
		% Acc>dec profile is saturated at VminSatLimit
		dt_23 = (dx - (2.0 * v_max ^ 2 - v_f ^ 2) / (2.0 * a_l)) / v_max;
		dt_l_hat = (2.0 * v_max - v_f) / a_l + dt_23;
		
		% Dec>acc profile is not saturated at VmaxSatLimit
		v_p = sqrt((2.0 * v_1 ^ 2 + v_f ^ 2) / 2.0 - dx * a_u);
		dt_u_hat = (2.0 * v_1 + v_f - 2.0 * v_p) / a_u;
	end
	
	% Determine the time difference at both saturation limits
	dt_u_tilde = dt_u_hat - dt_u; % For the same acceleration, a_u, the Acc>dec profile is always quicker than the Dec>acc profile
	dt_l_tilde = dt_l - dt_l_hat;
	
	% Compute reduction constants
	c_dtA = 2.0 * v_max - v_f;
	c_dxA = (2.0 * v_max ^ 2 - v_f ^ 2) / (2.0 * v_max);
	c_dtD = 2.0 * v_1 + v_f - 2.0 * v_min;
	c_dxD = (2.0 * v_1 ^ 2 + v_f ^ 2 - 2.0 * v_min ^ 2) / (2.0 * v_min);
	
	% There are four cases to consider given the time difference
	% 1. dt_tilde >= dt_u_tilde && dt_tilde >= dt_l_tilde
	% 2. dt_tilde  < dt_u_tilde && dt_tilde >= dt_l_tilde
	% 3. dt_tilde >= dt_u_tilde && dt_tilde  < dt_l_tilde
	% 4. dt_tilde  < dt_u_tilde && dt_tilde  < dt_l_tilde
	
	requires2ndOrderSolution = false;
	
	% 1. Time difference exceeds both saturation limits
	if (dt_tilde >= dt_u_tilde) && (dt_tilde >= dt_l_tilde)
		% Compute the acceleration
		solution.accDec.move = PATH_ACC_DEC_SATURATED;
		solution.decAcc.move = PATH_DEC_ACC_SATURATED;
		
		solution.accDec.a = (c_dtD - c_dtA - (c_dxD - c_dxA)) / (dt_tilde - dx * (1.0 / v_min - 1.0 / v_max));
		solution.decAcc.a = solution.accDec.a;
		solution.case = 1;
		
	% 2. Time difference below v_max saturation limit
	elseif (dt_tilde < dt_u_tilde) && (dt_tilde >= dt_l_tilde)
		% Prepare for second order solution
		solution.accDec.move = PATH_ACC_DEC_PEAK;
		solution.decAcc.move = PATH_DEC_ACC_SATURATED;
		
		c_1 = (v_f ^ 2) / 2.0;
		c_2 = c_dtD - c_dxD + v_f;
		c_3 = dt_tilde - (dx / v_min);
		p_2 = c_3 ^ 2;
		p_1 = - 4.0 * dx - 2.0 * c_2 * c_3;
		p_0 = c_2 ^ 2 - 4.0 * c_1;
		requires2ndOrderSolution = true;
		solution.case = 2;
		
	% 3. Time difference below v_min saturation limit
	elseif (dt_tilde >= dt_u_tilde) && (dt_tilde < dt_l_tilde)
		% Prepare for second order solution
		solution.accDec.move = PATH_ACC_DEC_SATURATED;
		solution.decAcc.move = PATH_DEC_ACC_PEAK;
		
		c_1 = (2.0 * v_1 ^ 2 + v_f ^ 2) / 2.0;
		c_2 = 2.0 * v_1 + v_f - (c_dtA - c_dxA);
		c_3 = dt_tilde + (dx / v_max);
		p_2 = c_3 ^ 2;
		p_1 = 4.0 * dx - 2.0 * c_2 * c_3;
		p_0 = c_2 ^ 2 - 4.0 * c_1;
		requires2ndOrderSolution = true;
		solution.case = 3;
		
	% 4. Time difference below both saturation limits
	else
		% Use the saturated acceleration limit from the smaller of the saturated time difference limits
		if dt_u_tilde < dt_l_tilde % a_u < a_l
			solution.accDec.move = PATH_ACC_DEC_SATURATED;
			solution.decAcc.move = PATH_DEC_ACC_PEAK;
			solution.accDec.a = a_u;
			solution.decAcc.a = a_u;
			
		else % a_u >= a_l
			solution.accDec.move = PATH_ACC_DEC_PEAK;
			solution.decAcc.move = PATH_DEC_ACC_SATURATED;
			solution.accDec.a = a_l;
			solution.decAcc.a = a_l;
		end
		solution.case = 4;
	end % dt_tilde cases
	
	if requires2ndOrderSolution
		[rootsSolution, rootsValid] = PathRoots(p_2, p_1, p_0);
		
		if !rootsValid
			printf("PathAccInTimeDiffWithRise call failed: Invalid roots for peak movement\n");
			valid = false;
			return;
			
		else
			if max(rootsSolution.r_1, rootsSolution.r_2) > 0.0 
				solution.accDec.a = max(rootsSolution.r_1, rootsSolution.r_2);
				solution.decAcc.a = solution.accDec.a;
			else
				printf("PathAccInTimeDiffWithRise call failed: Non-position 2nd order roots solution\n");
			end % Positive root?
		end % Valid roots?
	end % Required 2nd order solution?
	
	solution.accDec.dx 		= dx;
	solution.accDec.v(1) 	= 0.0;
	solution.accDec.v(2) 	= v_1;
	solution.accDec.t(1) 	= 0.0;
	solution.accDec.t(2) 	= v_1 / solution.accDec.a;
	if solution.accDec.move == PATH_ACC_DEC_SATURATED
		solution.accDec.v(3) = v_max;
		solution.accDec.v(4) = v_max;
		solution.accDec.v(5) = v_f;
		solution.accDec.t(3) = v_max / solution.accDec.a;
		solution.accDec.t(4) = v_max / solution.accDec.a + (dx - ((2.0 * v_max ^ 2 - v_f ^ 2) / (2.0 * solution.accDec.a))) / v_max;
		solution.accDec.t(5) = v_max / solution.accDec.a + (dx - ((2.0 * v_max ^ 2 - v_f ^ 2) / (2.0 * solution.accDec.a))) / v_max + (v_max - v_f) / solution.accDec.a;
	else
		v_p = sqrt(dx * solution.accDec.a + (v_f ^ 2) / 2.0);
		solution.accDec.v(3) = v_p;
		solution.accDec.v(4) = v_p;
		solution.accDec.v(5) = v_f;
		solution.accDec.t(3) = v_p / solution.accDec.a;
		solution.accDec.t(4) = v_p / solution.accDec.a;
		solution.accDec.t(5) = v_p / solution.accDec.a + (v_p - v_f) / solution.accDec.a;
	end
	
	solution.decAcc.dx 		= dx;
	solution.decAcc.v(1) 	= 0.0;
	solution.decAcc.v(2) 	= v_1;
	solution.decAcc.t(1) 	= 0.0;
	solution.decAcc.t(2) 	= v_1 / solution.decAcc.a;
	if solution.decAcc.move == PATH_DEC_ACC_SATURATED
		solution.decAcc.v(3) = v_min;
		solution.decAcc.v(4) = v_min;
		solution.decAcc.v(5) = v_f;
		solution.decAcc.t(3) = (2.0 * v_1 - v_min) / solution.decAcc.a;
		solution.decAcc.t(4) = (2.0 * v_1 - v_min) / solution.decAcc.a + (dx - ((2.0 * v_1 ^ 2 + v_f ^ 2 - 2.0 * v_min^2) / (2.0 * solution.decAcc.a))) / v_min;
		solution.decAcc.t(5) = (2.0 * v_1 - v_min) / solution.decAcc.a + (dx - ((2.0 * v_1 ^ 2 + v_f ^ 2 - 2.0 * v_min^2) / (2.0 * solution.decAcc.a))) / v_min + (v_f - v_min) / solution.decAcc.a;
	else
		v_p = sqrt((2.0 * v_1 ^ 2 + v_f ^ 2) / 2.0 - dx * solution.decAcc.a);
		solution.decAcc.v(3) = v_p;
		solution.decAcc.v(4) = v_p;
		solution.decAcc.v(5) = v_f;
		solution.decAcc.t(3) = (2.0 * v_1 - v_p) / solution.decAcc.a;
		solution.decAcc.t(4) = (2.0 * v_1 - v_p) / solution.decAcc.a;
		solution.decAcc.t(5) = (2.0 * v_1 + v_f - 2.0 * v_p) / solution.decAcc.a;
	end
	
	% Validate the result
	valid = true;
	
	if printResult
		printf("PathAccInTimeDiffWithRise call: a = %.3f, Case %d, Moves %d, %d\n", solution.accDec.a, solution.case, solution.accDec.move, solution.decAcc.move);
	end
	
end % Function
