%!octave

function [Solution, Valid] = GetAccInTimeDiff(tdiff, dx, v0, vf, vmin, vmax, PrintResult = false)
	% GetAccInTimeDiff(tdiff, dx, v0, vf, vmin, vmax, PrintResult = false)
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
	Solution.vmax.t 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmax.dx 	= 0.0;
	Solution.vmax.v 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmax.a 	= 0.0;
	Solution.vmax.Move 	= PATH_MOVE_NONE;
	Solution.vmin.t 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmin.dx 	= 0.0;
	Solution.vmin.v 	= [0.0, 0.0, 0.0, 0.0];
	Solution.vmin.a 	= 0.0;
	Solution.vmin.Move 	= PATH_MOVE_NONE;
	
	% Input requirements
	% #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("GetAccInTimeDiff call failed: Implausible velocity limits %.3f, %.3f\n", vmin, vmax); 
		Valid = false; 
		return;
	
	% #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("GetAccInTimeDiff call failed: Endpoint velocities %.3f, %.3f exceed limits %.3f, %.3f\n", v0, vf, vmin, vmax); 
		Valid = false; 
		return;
	
	% #3: Positive time and distance
	elseif (tdiff <= 0.0) || (dx <= 0.0)
		printf("GetAccInTimeDiff call failed: Time difference or distance non-positive %.3f, %.3f\n", tdiff, dx); 
		Valid = false; 
		return;
		
	% #4: Valid distance given velocity limits
	elseif (tdiff >= (dx / vmin - dx / vmax))
		printf("GetAccInTimeDiff call failed: Impossible time difference %.3f given velocity limits %.3f\n", tdiff, dx / vmin - dx / vmax); 
		Valid = false; 
		return;
		
	end % Requirements
	
	% Determine the saturated acceleration limit foreach profile
	AccVmaxSatLimit = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * dx); % Accelerations higher than this will saturate the vmax profile
	AccVminSatLimit = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	
	% Determine the time duration at the saturation acceleration limit
	TimeVmaxSatLimit = (2.0 * vmax - v0 - vf) / AccVmaxSatLimit; % Any time duration larger that this will saturate the vmax profile
	TimeVminSatLimit = (v0 + vf - 2.0 * vmin) / AccVminSatLimit;
	
	% Determine the time duration at the alternative profile when saturated
	if AccVminSatLimit < AccVmaxSatLimit
		% The vmax profile is not saturated
		PeakVel = sqrt(dx * AccVminSatLimit + (v0 ^ 2 + vf ^ 2) / 2.0);
		VmaxTimeAtVminSatLimit = (2.0 * PeakVel - v0 - vf) / AccVminSatLimit;
	else
		% The vmax profile is saturated
		SaturatedTime = (dx - (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * AccVminSatLimit)) / vmax;
		VmaxTimeAtVminSatLimit = (2.0 * vmax - v0 - vf) / AccVminSatLimit + SaturatedTime;
	end % Vmin saturated acc limit versus vmax
	
	if AccVmaxSatLimit < AccVminSatLimit
		% The vmin profile is not saturated
		DipVel = sqrt(dx * AccVmaxSatLimit - (v0 ^ 2 + vf ^ 2) / 2.0);
		VminTimeAtVmaxSatLimit = (v0 + vf - 2.0 * DipVel) / AccVmaxSatLimit;
	else
		% The vmin profile is saturated
		SaturatedTime = (dx - (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * AccVmaxSatLimit)) / vmin;
		VminTimeAtVmaxSatLimit = (v0 + vf - 2.0 * vmin) / AccVmaxSatLimit + SaturatedTime;
	end % Vmax saturated acc limit verus vmin
	
	% Determine the time differences at the saturation limits
	TimeDiffAtVmaxSatLimit = VminTimeAtVmaxSatLimit - TimeVmaxSatLimit; % For the same acceleration, the vmin profile time duration is always greater than the vmax profile time
	TimeDiffAtVminSatLimit = TimeVminSatLimit - VmaxTimeAtVminSatLimit;
	
	% Equate reduction constants
	cVmaxTime 		= 2.0 * vmax - v0 - vf;
	cVminTime 		= v0 + vf - 2.0 * vmin;
	cVmaxDistance 	= (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * vmax);
	cVminDistance 	= (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * vmin);
	
	% There are four cases to consider given the timer difference input
	% 1. Dt >= Dt_vmax_sat && Dt >= Dt_vmin_sat
	% 2. Dt <  Dt_vmax_sat && Dt >= Dt_vmin_sat
	% 3. Dt >= Dt_vmax_sat && Dt <  Dt_vmin_sat
	% 4. Dt <  Dt_vmax_sat && Dt <  Dt_vmin_sat
	
	RequiresSecondOrderSolution = false;
	
	% 1. Time difference exceeds both saturation limits
	if (tdiff >= TimeDiffAtVmaxSatLimit) && (tdiff >= TimeDiffAtVminSatLimit)
		% Determine the final solution
		Solution.vmax.Move 	= PATH_ACC_DEC_SATURATED;
		Solution.vmax.a 	= (cVminTime - cVmaxTime - (cVminDistance - cVmaxDistance)) / (tdiff - ((dx / vmin) - (dx / vmax)));
		Solution.vmin.Move 	= PATH_DEC_ACC_SATURATED;
		Solution.vmin.a 	= Solution.vmax.a; % Same acceleration for each profile
		
	elseif (tdiff < TimeDiffAtVmaxSatLimit) && (tdiff >= TimeDiffAtVminSatLimit)
		% Equate reduction constants
		c1 = (v0 ^ 2 + vf ^ 2) / 2.0;
		c2 = cVminTime - cVminDistance + v0 + vf;
		c3 = tdiff - (dx / vmin);
		p2 = c3 ^ 2;
		p1 = - 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 + 4.0 * c1;
		
		Solution.vmax.Move = PATH_ACC_DEC_PEAK;
		Solution.vmin.Move = PATH_DEC_ACC_SATURATED;
		
		RequiresSecondOrderSolution = true;
		
	elseif (tdiff >= TimeDiffAtVmaxSatLimit) && (tdiff < TimeDiffAtVmaxSatLimit)
		% Equate reduction constants
		c1 = (v0 ^ 2 + vf ^ 2) / 2.0;
		c2 = (-1.0) * cVminTime + cVmaxTime + v0 + vf;
		c3 = tdiff + (dx / vmax);
		p2 = c3 ^ 2;
		p1 = 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 - 4.0 * c1;
		
		Solution.vmax.Move = PATH_ACC_DEC_SATURATED;
		Solution.vmin.Move = PATH_DEC_ACC_PEAK;
		
		RequiresSecondOrderSolution = true;
		
	else
		% Use the acceleration from the smaller time difference at a saturation limit
		if TimeDiffAtVmaxSatLimit < TimeDiffAtVminSatLimit
			% Use the acceleration at the vmax saturation limit
			Solution.vmax.a 	= AccVmaxSatLimit;
			Solution.vmax.Move 	= PATH_ACC_DEC_SATURATED;
			Solution.vmin.a 	= Solution.vmax.a;
			Solution.vmin.Move 	= PATH_DEC_ACC_PEAK;
			
		else
			% Use the acceleration at the vmin saturation limit
			Solution.vmin.a 	= AccVminSatLimit;
			Solution.vmin.Move 	= PATH_DEC_ACC_SATURATED;
			Solution.vmax.a 	= Solution.vmin.a;
			Solution.vmax.Move 	= PATH_ACC_DEC_PEAK;
			
		end
	end
	
	if RequiresSecondOrderSolution
		[RootsSolution, RootsValid] = SecondOrderRoots(p2, p1, p0);
		
		if !RootsValid
			printf("GetAccInTimeDiff call failed: Invalid roots for peak movement\n");
			Valid = false;
			return;
			
		else
			if max(RootsSolution.r1, RootsSolution.r2) > 0
				Solution.vmax.a = max(RootsSolution.r1, RootsSolution.r2);
				Solution.vmin.a = Solution.vmax.a;
				
			else
				printf("GetAccInTimeDiff call failed: Non-positive solution from second order equation\n");
				Valid = false;
				return;
			end % Positive root?
		end % Valid roots?
	end % Second order solution?
	
	Solution.vmax.dx = dx;
	if Solution.vmax.Move == PATH_ACC_DEC_SATURATED
		Solution.vmax.v(1) = v0;
		Solution.vmax.v(2) = vmax;
		Solution.vmax.v(3) = vmax;
		Solution.vmax.v(4) = vf;
		Solution.vmax.t(1) = 0.0;
		Solution.vmax.t(2) = (vmax - v0) / Solution.vmax.a;
		Solution.vmax.t(3) = (vmax - v0) / Solution.vmax.a + (dx - ((2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * Solution.vmax.a))) / vmax;
		Solution.vmax.t(4) = (vmax - v0) / Solution.vmax.a + (dx - ((2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * Solution.vmax.a))) / vmax + (vmax - vf) / Solution.vmax.a;
	else
		PeakVel = sqrt(dx * Solution.vmax.a + (v0 ^ 2 + vf ^ 2) / 2.0);
		Solution.vmax.v(1) = v0;
		Solution.vmax.v(2) = PeakVel;
		Solution.vmax.v(3) = PeakVel;
		Solution.vmax.v(4) = vf;
		Solution.vmax.t(1) = 0.0;
		Solution.vmax.t(2) = (PeakVel - v0) / Solution.vmax.a;
		Solution.vmax.t(3) = (PeakVel - v0) / Solution.vmax.a;
		Solution.vmax.t(4) = (PeakVel - v0) / Solution.vmax.a + (PeakVel - vf) / Solution.vmax.a;
	end
	
	Solution.vmin.dx = dx;
	if Solution.vmin.Move == PATH_DEC_ACC_SATURATED
		Solution.vmin.v(1) = v0;
		Solution.vmin.v(2) = vmin;
		Solution.vmin.v(3) = vmin;
		Solution.vmin.v(4) = vf;
		Solution.vmin.t(1) = 0.0;
		Solution.vmin.t(2) = (v0 - vmin) / Solution.vmin.a;
		Solution.vmin.t(3) = (v0 - vmin) / Solution.vmin.a + (dx - ((v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * Solution.vmin.a))) / vmin;
		Solution.vmin.t(4) = (v0 - vmin) / Solution.vmin.a + (dx - ((v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * Solution.vmin.a))) / vmin + (vf - vmin) / Solution.vmin.a;
	else
		DipVel = sqrt((v0 ^ 2 + vf ^ 2) / 2.0 - dx * Solution.vmin.a);
		Solution.vmin.v(1) = v0;
		Solution.vmin.v(2) = DipVel;
		Solution.vmin.v(3) = DipVel;
		Solution.vmin.v(4) = vf;
		Solution.vmin.t(1) = 0.0;
		Solution.vmin.t(2) = (v0 - DipVel) / Solution.vmin.a;
		Solution.vmin.t(3) = (v0 - DipVel) / Solution.vmin.a;
		Solution.vmin.t(4) = (v0 - DipVel) / Solution.vmin.a + (vf - DipVel) / Solution.vmin.a;
	end
	
	% Validate the solution
	Valid = true;
	
	if PrintResult
		printf("GetAccInTimeDiff call: Acc %.3f, Cases %d, %d\n", Solution.vmin.a, Solution.vmin.Move, Solution.vmax.Move);
	end
	
end % Function
