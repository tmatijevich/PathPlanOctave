%!octave

function [solution, valid] = PathAccInTime(dt_tilde, dx, v_0, v_f, vmin, v_max, PrintResult = false)
	% PathAccInTime(dt_tilde, dx, v_0, v_f, vmin, v_max, PrintResult = false)
	% Determine the minimum acceleration required to achieve both movement extremes
	% within a specific time difference
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
	solution.v_max.t 	= [0.0, 0.0, 0.0, 0.0];
	solution.v_max.dx 	= 0.0;
	solution.v_max.v 	= [0.0, 0.0, 0.0, 0.0];
	solution.v_max.a 	= 0.0;
	solution.v_max.Move 	= PATH_MOVE_NONE;
	solution.vmin.t 	= [0.0, 0.0, 0.0, 0.0];
	solution.vmin.dx 	= 0.0;
	solution.vmin.v 	= [0.0, 0.0, 0.0, 0.0];
	solution.vmin.a 	= 0.0;
	solution.vmin.Move 	= PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (v_max <= vmin)
		printf("PathAccInTime call failed: Implausible velocity limits %.3f, %.3f\n", vmin, v_max); 
		valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v_0 < vmin) || (v_0 > v_max) || (v_f < vmin) || (v_f > v_max)
		printf("PathAccInTime call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v_0, v_f, vmin, v_max); 
		valid = false; 
		return;
	
	% #3: Positive time and distance
	elseif (dt_tilde <= 0.0) || (dx <= 0.0)
		printf("PathAccInTime call failed: Time difference or distance non-positive %.3f, %.3f\n", dt_tilde, dx); 
		valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (dt_tilde >= (dx / vmin - dx / v_max))
		printf("PathAccInTime call failed: Impossible time difference %.3f given velocity limits %.3f\n", dt_tilde, dx / vmin - dx / v_max); 
		valid = false; 
		return;
		
	end % Requirements
	
	% Determine the saturated acceleration limit foreach profile
	AccVmaxSatLimit = (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * dx); % Accelerations higher than this will saturate the v_max profile
	AccVminSatLimit = (v_0 ^ 2 + v_f ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	
	% Determine the time duration at the saturation acceleration limit
	TimeVmaxSatLimit = (2.0 * v_max - v_0 - v_f) / AccVmaxSatLimit; % Any time duration larger that this will saturate the v_max profile
	TimeVminSatLimit = (v_0 + v_f - 2.0 * vmin) / AccVminSatLimit;
	
	% Determine the time duration at the alternative profile when saturated
	if AccVminSatLimit < AccVmaxSatLimit
		% The v_max profile is not saturated
		PeakVel = sqrt(dx * AccVminSatLimit + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		VmaxTimeAtVminSatLimit = (2.0 * PeakVel - v_0 - v_f) / AccVminSatLimit;
	else
		% The v_max profile is saturated
		SaturatedTime = (dx - (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * AccVminSatLimit)) / v_max;
		VmaxTimeAtVminSatLimit = (2.0 * v_max - v_0 - v_f) / AccVminSatLimit + SaturatedTime;
	end % Vmin saturated acc limit versus v_max
	
	if AccVmaxSatLimit < AccVminSatLimit
		% The vmin profile is not saturated
		DipVel = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * AccVmaxSatLimit);
		VminTimeAtVmaxSatLimit = (v_0 + v_f - 2.0 * DipVel) / AccVmaxSatLimit;
	else
		% The vmin profile is saturated
		SaturatedTime = (dx - (v_0 ^ 2 + v_f ^ 2 - 2.0 * vmin ^ 2) / (2.0 * AccVmaxSatLimit)) / vmin;
		VminTimeAtVmaxSatLimit = (v_0 + v_f - 2.0 * vmin) / AccVmaxSatLimit + SaturatedTime;
	end % Vmax saturated acc limit verus vmin
	
	% Determine the time differences at the saturation limits
	TimeDiffAtVmaxSatLimit = VminTimeAtVmaxSatLimit - TimeVmaxSatLimit; % For the same acceleration, the vmin profile time duration is always greater than the v_max profile time
	TimeDiffAtVminSatLimit = TimeVminSatLimit - VmaxTimeAtVminSatLimit;
	
	% Equate reduction constants
	cVmaxTime 		= 2.0 * v_max - v_0 - v_f;
	cVminTime 		= v_0 + v_f - 2.0 * vmin;
	cVmaxDistance 	= (2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * v_max);
	cVminDistance 	= (v_0 ^ 2 + v_f ^ 2 - 2.0 * vmin ^ 2) / (2.0 * vmin);
	
	% There are four cases to consider given the timer difference input
	% 1. Dt >= Dt_vmax_sat && Dt >= Dt_vmin_sat
	% 2. Dt <  Dt_vmax_sat && Dt >= Dt_vmin_sat
	% 3. Dt >= Dt_vmax_sat && Dt <  Dt_vmin_sat
	% 4. Dt <  Dt_vmax_sat && Dt <  Dt_vmin_sat
	
	RequiresSecondOrderSolution = false;
	
	% 1. Time difference exceeds both saturation limits
	if (dt_tilde >= TimeDiffAtVmaxSatLimit) && (dt_tilde >= TimeDiffAtVminSatLimit)
		% Determine the final solution
		solution.v_max.Move 	= PATH_ACC_DEC_SATURATED;
		solution.v_max.a 	= (cVminTime - cVmaxTime - (cVminDistance - cVmaxDistance)) / (dt_tilde - ((dx / vmin) - (dx / v_max)));
		solution.vmin.Move 	= PATH_DEC_ACC_SATURATED;
		solution.vmin.a 	= solution.v_max.a; % Same acceleration for each profile
		
	elseif (dt_tilde < TimeDiffAtVmaxSatLimit) && (dt_tilde >= TimeDiffAtVminSatLimit)
		% Equate reduction constants
		c1 = (v_0 ^ 2 + v_f ^ 2) / 2.0;
		c2 = cVminTime - cVminDistance + v_0 + v_f;
		c3 = dt_tilde - (dx / vmin);
		p2 = c3 ^ 2;
		p1 = - 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 - 4.0 * c1;
		
		solution.v_max.Move = PATH_ACC_DEC_PEAK;
		solution.vmin.Move = PATH_DEC_ACC_SATURATED;
		
		RequiresSecondOrderSolution = true;
		
	elseif (dt_tilde >= TimeDiffAtVmaxSatLimit) && (dt_tilde < TimeDiffAtVminSatLimit)
		% Equate reduction constants
		c1 = (v_0 ^ 2 + v_f ^ 2) / 2.0;
		c2 = (-1.0) * cVmaxTime + cVmaxDistance + v_0 + v_f;
		c3 = dt_tilde + (dx / v_max);
		p2 = c3 ^ 2;
		p1 = 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 - 4.0 * c1;
		
		solution.v_max.Move = PATH_ACC_DEC_SATURATED;
		solution.vmin.Move = PATH_DEC_ACC_PEAK;
		
		RequiresSecondOrderSolution = true;
		
	else
		% Use the acceleration from the smaller time difference at a saturation limit
		if TimeDiffAtVmaxSatLimit < TimeDiffAtVminSatLimit
			% Use the acceleration at the v_max saturation limit
			solution.v_max.a 	= AccVmaxSatLimit;
			solution.v_max.Move 	= PATH_ACC_DEC_SATURATED;
			solution.vmin.a 	= solution.v_max.a;
			solution.vmin.Move 	= PATH_DEC_ACC_PEAK;
			
		else
			% Use the acceleration at the vmin saturation limit
			solution.vmin.a 	= AccVminSatLimit;
			solution.vmin.Move 	= PATH_DEC_ACC_SATURATED;
			solution.v_max.a 	= solution.vmin.a;
			solution.v_max.Move 	= PATH_ACC_DEC_PEAK;
			
		end
	end
	
	if RequiresSecondOrderSolution
		[RootsSolution, RootsValid] = SecondOrderRoots(p2, p1, p0);
		
		if !RootsValid
			printf("PathAccInTime call failed: Invalid roots for peak movement\n");
			valid = false;
			return;
			
		else
			if max(RootsSolution.r1, RootsSolution.r2) > 0
				solution.v_max.a = max(RootsSolution.r1, RootsSolution.r2);
				solution.vmin.a = solution.v_max.a;
				
			else
				printf("PathAccInTime call failed: Non-positive solution from second order equation\n");
				valid = false;
				return;
			end % Positive root?
		end % Valid roots?
	end % Second order solution?
	
	solution.v_max.dx = dx;
	if solution.v_max.Move == PATH_ACC_DEC_SATURATED
		solution.v_max.v(1) = v_0;
		solution.v_max.v(2) = v_max;
		solution.v_max.v(3) = v_max;
		solution.v_max.v(4) = v_f;
		solution.v_max.t(1) = 0.0;
		solution.v_max.t(2) = (v_max - v_0) / solution.v_max.a;
		solution.v_max.t(3) = (v_max - v_0) / solution.v_max.a + (dx - ((2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * solution.v_max.a))) / v_max;
		solution.v_max.t(4) = (v_max - v_0) / solution.v_max.a + (dx - ((2.0 * v_max ^ 2 - v_0 ^ 2 - v_f ^ 2) / (2.0 * solution.v_max.a))) / v_max + (v_max - v_f) / solution.v_max.a;
	else
		PeakVel = sqrt(dx * solution.v_max.a + (v_0 ^ 2 + v_f ^ 2) / 2.0);
		solution.v_max.v(1) = v_0;
		solution.v_max.v(2) = PeakVel;
		solution.v_max.v(3) = PeakVel;
		solution.v_max.v(4) = v_f;
		solution.v_max.t(1) = 0.0;
		solution.v_max.t(2) = (PeakVel - v_0) / solution.v_max.a;
		solution.v_max.t(3) = (PeakVel - v_0) / solution.v_max.a;
		solution.v_max.t(4) = (PeakVel - v_0) / solution.v_max.a + (PeakVel - v_f) / solution.v_max.a;
	end
	
	solution.vmin.dx = dx;
	if solution.vmin.Move == PATH_DEC_ACC_SATURATED
		solution.vmin.v(1) = v_0;
		solution.vmin.v(2) = vmin;
		solution.vmin.v(3) = vmin;
		solution.vmin.v(4) = v_f;
		solution.vmin.t(1) = 0.0;
		solution.vmin.t(2) = (v_0 - vmin) / solution.vmin.a;
		solution.vmin.t(3) = (v_0 - vmin) / solution.vmin.a + (dx - ((v_0 ^ 2 + v_f ^ 2 - 2.0 * vmin ^ 2) / (2.0 * solution.vmin.a))) / vmin;
		solution.vmin.t(4) = (v_0 - vmin) / solution.vmin.a + (dx - ((v_0 ^ 2 + v_f ^ 2 - 2.0 * vmin ^ 2) / (2.0 * solution.vmin.a))) / vmin + (v_f - vmin) / solution.vmin.a;
	else
		DipVel = sqrt((v_0 ^ 2 + v_f ^ 2) / 2.0 - dx * solution.vmin.a);
		solution.vmin.v(1) = v_0;
		solution.vmin.v(2) = DipVel;
		solution.vmin.v(3) = DipVel;
		solution.vmin.v(4) = v_f;
		solution.vmin.t(1) = 0.0;
		solution.vmin.t(2) = (v_0 - DipVel) / solution.vmin.a;
		solution.vmin.t(3) = (v_0 - DipVel) / solution.vmin.a;
		solution.vmin.t(4) = (v_0 - DipVel) / solution.vmin.a + (v_f - DipVel) / solution.vmin.a;
	end
	
	% Validate the solution
	valid = true;
	
	if PrintResult
		printf("PathAccInTime call: Acc %.3f, Cases %d, %d\n", solution.vmin.a, solution.vmin.Move, solution.v_max.Move);
	end
	
end % Function
