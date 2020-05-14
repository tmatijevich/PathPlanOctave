%!octave

function [soln, valid] = Kin_GetAccInTimespanPlus(tdiff, dx, v1, vf, vmin, vmax)
	% Determine the minimum acceleration required to achieve movement extremes within the specified timespan
	% This function assumes positive kinematic values and infinite jerk
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Assume the result will be valid
	valid = true;
	soln.a = 0.0; soln.cs = 0; % Fallback/invalid result
	
	% Condition #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("Kin_GetAccInTimespanPlus call invalid: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); valid = false; return;
	
	% Condition #2: Endpoint velocities within limits
	elseif (v1 < vmin) || (v1 > vmax) || (vf < vmin) || (vf > vmax)
		printf("Kin_GetAccInTimespanPlus call invalid: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v1, vf, vmin, vmax); valid = false; return;
	
	% Condition #3: Positive time difference and distance
	elseif (tdiff <= 0.0) || (dx <= 0.0)
		printf("Kin_GetAccInTimespanPlus call invalid: Time difference or distance non-positive %1.3f, %1.3f\n", tdiff, dx); valid = false; return;
	
	% Condition #4: Valid time difference given velocity limits
	elseif (tdiff >= (dx / vmin - dx / vmax))
		printf("Kin_GetAccInTimespanPlus call invalid: Impossible time difference %1.3f given velocity limits %1.3f\n", tdiff, dx / vmin - dx / vmax); valid = false; return;
	
	end % Conditions
	
	% Inflection constants - given dx, what timespans exceed the inflection points?
	aVmaxInflection = (2.0 * vmax ^ 2 - vf ^ 2) / (2.0 * dx);
	aVminInflection = (2.0 * v1 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * dx);
	
	tVmaxInflection = (2.0 * vmax - vf) / aVmaxInflection;
	tVminInflection = (2.0 * v1 + vf - 2.0 * vmin) / aVminInflection;
	
	% tVmax @ VminInflection
	if aVminInflection < aVmaxInflection
		vpeak = sqrt(dx * aVminInflection + (vf ^ 2) / 2.0);
		tVmaxAtVminInflection = (2.0 * vpeak - vf) / aVminInflection;
	else
		t12vmax = (dx - (2.0 * vmax ^ 2 - vf ^ 2) / (2.0 * aVminInflection)) / vmax;
		tVmaxAtVminInflection = (2.0 * vmax - vf) / aVminInflection + t12vmax;
	end
	% tVmin @ VmaxInflection
	if aVmaxInflection < aVminInflection
		vdip = sqrt(v1 ^ 2 + (vf ^ 2) / 2.0 - dx * aVmaxInflection);
		tVminAtVmaxInflection = (2.0 * v1 + vf - 2.0 * vdip) / aVmaxInflection;
	else
		t12vmin = (dx - (2.0 * v1 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * aVmaxInflection)) / vmin;
		tVminAtVmaxInflection = (2.0 * v1 + vf - 2.0 * vmin) / aVmaxInflection + t12vmin;
	end
	
	% Under condition #3 (tdiff > 0.0) ==> (tVminAtVmaxInflection > tVmaxInflection) && (tVminInflection > tVmaxAtVminInflection)
	tdiffVmaxInflection = tVminAtVmaxInflection - tVmaxInflection;
	tdiffVminInflection = tVminInflection - tVmaxAtVminInflection;
	
	% Constants for all three cases
	cVmax_dt = 2.0 * vmax - vf;
	cVmax_dx = (2.0 * vmax ^ 2 - vf ^ 2) / (2.0 * vmax);
	cVmin_dt = 2.0 * v1 + vf - 2.0 * vmin;
	cVmin_dx = (2.0 * v1 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * vmin);
	
	if (tdiff >= tdiffVmaxInflection) && (tdiff >= tdiffVminInflection)
		soln.cs = 22;
		soln.a = (cVmin_dx - cVmin_dt - (cVmax_dx - cVmax_dt)) / ((dx / vmin - dx / vmax) - tdiff);
	
	elseif (tdiff >= tdiffVminInflection)
		soln.cs = 12;
		c1 = (vf ^ 2) / 2.0;
		c2 = cVmin_dt - cVmin_dx + vf;
		c3 = tdiff - (dx / vmin);
		p2 = c3 ^ 2;
		p1 = - 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 - 4.0 * c1;
		
	elseif (tdiff >= tdiffVmaxInflection)
		soln.cs = 21;
		c1 = v1 ^ 2 + (vf ^ 2) / 2.0;
		c2 = 2.0 * v1 + vf - (cVmax_dt - cVmax_dx);
		c3 = tdiff + (dx / vmax);
		p2 = c3 ^ 2;
		p1 = 4.0 * dx - 2.0 * c2 * c3;
		p0 = c2 ^ 2 - 4.0 * c1;
		
	else
		soln.cs = 11;
		if (tdiffVmaxInflection < tdiffVminInflection)
			soln.a = aVmaxInflection;
		else
			soln.a = aVminInflection;
		endif
		printf("fnGetMinAccelInTimespanPlus call warning: Requires higher order solver, using sufficient accel. %2.3f\n", soln.a); return;
		
	end % Timespan case
	
	if (soln.cs == 12) || (soln.cs == 21)
		[solnRoots, validRoots] = Math_2ndOrderRoots(p2, p1, p0);
		if !validRoots
			printf("Kin_GetAccInTimespanPlus call invalid: Imaginary roots for case %d\n", soln.cs); valid = false; return;
			
		else
			if max(solnRoots.r1, solnRoots.r2) > 0.0
				soln.a = max(solnRoots.r1, solnRoots.r2);
			else
				printf("Kin_GetAccInTimespanPlus call invalid: Invalid roots %1.3f, %1.3f for case %d\n", solnRoots.r1, solnRoots.r2, soln.cs); valid = false; return;
			end % Positive root
		end % Imaginary roots
	end % Triangle profile
	
	printf("Kin_GetAccInTimespanPlus call: Acc %2.3f, Case %d\n", soln.a, soln.cs);
	
end
