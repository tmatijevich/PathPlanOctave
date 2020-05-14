%!octave

function [soln, valid] = Kin_GetTimespan(dx, v0, vf, vmin, vmax, a)
	% Determine the window of time between the time minimizing and time maximizing velocity profiles
	% This function assumes positive kinematic values and infinite jerk
	% Date: 2020-03-25
	% Created by: Tyler Matijevich
	
	% Assume the result will be valid
	valid = true;
	soln.tspan = 0.0; % Fallback/invalid result
	soln.tVmax1 = 0.0; soln.tVmax2 = 0.0; soln.tVmax = 0.0; soln.v1max = 0.0;
	soln.tVmin1 = 0.0; soln.tVmin2 = 0.0; soln.tVmin = 0.0; soln.v1min = 0.0;
	
	% Condition #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("Kin_GetTimespan call invalid: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); valid = false; return;
	
	% Condition #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("Kin_GetTimespan call invalid: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); valid = false; return;
	
	% Condition #3: Positive time difference and distance
	elseif (dx <= 0.0) || (a <= 0.0)
		printf("Kin_GetTimespan call invalid: Distance or acceleration non-positive %1.3f, %2.3f\n", dx, a); valid = false; return;
	
	end % Conditions
	
	% Determine the time minimizing velocity maximizing profile
	dxVmaxInflection = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * a);
	if dx < dxVmaxInflection
		% Triangle profile
		v1 = sqrt(dx * a + (v0 ^ 2 + vf ^ 2) / 2.0); soln.v1max = v1;
		soln.tVmax1 = (v1 - v0) / a;
		soln.tVmax2 = soln.tVmax1;
		soln.tVmax = (2.0 * v1 - v0 - vf) / a;
		soln.cs = 10;
	else
		% Trapezoid profile
		tVmax12 = (dx - dxVmaxInflection) / vmax; soln.v1max = vmax;
		soln.tVmax1 = (vmax - v0) / a;
		soln.tVmax2 = soln.tVmax1 + tVmax12;
		soln.tVmax = soln.tVmax2 + (vmax - vf) / a;
		soln.cs = 20;
	end
	
	% Determine the time maximizing velocity minimizing profile
	dxVminInflection = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * a);
	if dx < dxVminInflection
		% Triangle profile
		v1 = sqrt((v0 ^ 2 + vf ^ 2) / 2.0 - dx * a); soln.v1min = v1;
		soln.tVmin1 = (v0 - v1) / a;
		soln.tVmin2 = soln.tVmin1;
		soln.tVmin = (v0 + vf - 2.0 * v1) / a;
		soln.cs = soln.cs + 1;
	else
		% Trapezoid profile
		tVmin12 = (dx - dxVminInflection) / vmin; soln.v1min = vmin;
		soln.tVmin1 = (v0 - vmin) / a;
		soln.tVmin2 = soln.tVmin1 + tVmin12;
		soln.tVmin = soln.tVmin2 + (vf - vmin) / a;
		soln.cs = soln.cs + 2;
	end
	
	printf("Kin_GetTimespan call: Timespan %1.3f = %1.3f - %1.3f, Case %d\n", soln.tVmin - soln.tVmax, soln.tVmin, soln.tVmax, soln.cs);
	
end
