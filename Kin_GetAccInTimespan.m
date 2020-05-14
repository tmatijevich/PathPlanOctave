%!octave

function [soln, valid] = Kin_GetAccInTimespan(tdiff, dx, v0, vf, vmin, vmax)
	% Determine the minimum acceleration required to achieve movement extremes within the specified timespan
	% This function assumes positive kinematic values and infinite jerk
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Assume the result will be valid
	valid = true;
	soln.a = 0.0; soln.cs = 0; % Fallback/invalid result
	
	% Condition #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("Kin_GetAccInTimespan call invalid: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); valid = false; return;
	
	% Condition #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("Kin_GetAccInTimespan call invalid: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); valid = false; return;
	
	% Condition #3: Positive time difference and distance
	elseif (tdiff <= 0.0) || (dx <= 0.0)
		printf("Kin_GetAccInTimespan call invalid: Time difference or distance non-positive %1.3f, %1.3f\n", tdiff, dx); valid = false; return;
	
	% Condition #4: Valid time difference given velocity limits
	elseif (tdiff >= (dx / vmin - dx / vmax))
		printf("Kin_GetAccInTimespan call invalid: Impossible time difference %1.3f given velocity limits %1.3f\n", tdiff, dx / vmin - dx / vmax); valid = false; return;
	
	end % Conditions
	
	% Constants
	% Maximum velocity, minimum time
	cVmax_dt = 2.0 * vmax - v0 - vf;
	cVmax_dx = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * vmax);
	
	% Minimum velocity, maximum time
	cVmin_dt = v0 + vf - 2.0 * vmin;
	cVmin_dx = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * vmin);
	
	% Test the solution of two saturated profiles
	a_test = ((cVmin_dx - cVmax_dx) - (cVmin_dt - cVmax_dt)) / ((dx / vmin - dx / vmax) - tdiff);
	if a_test <= 0.0
		printf("Kin_GetAccInTimespan call invalid: Implausible acceleration %1.3f\n", a_test); valid = false; return;
	end
	
	% Test the acceleration against the inflection points
	aVmaxInflection = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * dx);
	aVminInflection = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	
	if a_test < aVminInflection
		printf("Kin_GetAccInTimespan call invalid: Solution requires higher order solver\n"); valid = false; return;
	elseif a_test < aVmaxInflection
		% Calculate the triangle profile for maximizing velocity, minimizing time
		soln.cs = 12;
		
		c3 = cVmin_dt - cVmin_dx + v0 + vf;
		c4 = tdiff - dx / vmin;
		c5 = (v0 ^ 2 + vf ^ 2) / 2.0;
		
		[solnRoots, validRoots] = Math_2ndOrderRoots(0.25 * c4 ^ 2, - 0.5 * c3 * c4 - dx, 0.25 * c3 ^ 2 - c5);
		if !validRoots
			printf("Kin_GetAccInTimespan call invalid: Imaginary roots for case %d\n", soln.cs); valid = false; return;
		else
			if (solnRoots.r1 > 0.0) && (solnRoots.r1 == solnRoots.r2)
				soln.a = solnRoots.r1;
			elseif (solnRoots.r1 > 0.0) && (solnRoots.r2 < 0.0)
				soln.a = solnRoots.r1;
			elseif (solnRoots.r1 < 0.0) && (solnRoots.r2 > 0.0)
				soln.a = solnRoots.r2;
			else
				printf("Kin_GetAccInTimespan call invalid: Invalid roots for case %d\n", soln.cs); valid = false; return;
			end
		end
	else
		soln.cs = 22;
		soln.a = a_test;
	end
	
	printf("Kin_GetAccInTimespan call: Acc %1.3f, Case %d\n", soln.a, soln.cs);
	
end
