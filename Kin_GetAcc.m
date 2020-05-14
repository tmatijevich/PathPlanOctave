%!octave

function [soln, valid] = Kin_GetAcc(dt, dx, v0, vf, vmin, vmax)
	% Determine the minimum acceleration to change velocity in time over a distance
	% This function assumes positive kinematic values and infinite jerk
	% Date: 2020-04-10
	% Created by: Tyler Matijevich
	
	% Assume the result will be valid
	valid = true;
	soln.v = 0.0; soln.a = 0.0; soln.cs = 0; % Fallback/invalid result
	
	% Condition #1: Plausible velocity limits
	if (vmin < 0.0) || (vmax <= vmin)
		printf("Kin_GetAcc call invalid: Implausible velocity limits %1.3f, %1.3f\n", vmin, vmax); valid = false; return;
	
	% Condition #2: Endpoint velocities within limits
	elseif (v0 < vmin) || (v0 > vmax) || (vf < vmin) || (vf > vmax)
		printf("Kin_GetAcc call invalid: Endpoint velocities %1.3f, %1.3f exceed limits %1.3f, %1.3f\n", v0, vf, vmin, vmax); valid = false; return;
	
	% Condition #3: Positive time and distance
	elseif (dt <= 0.0) || (dx <= 0.0)
		printf("Kin_GetAcc call invalid: Time or distance non-positive %1.3f, %1.3f\n", dt, dx); valid = false; return;
		
	% Condition #4: Valid distance given velocity limits
	elseif (dx <= (vmin * dt)) || (dx >= (vmax * dt))
		printf("Kin_GetAcc call invalid: Impossible distance %1.3f given limits %1.3f, %1.3f\n", dx, vmin * dt, vmax * dt); valid = false; return;
		
	end % Conditions
	
	dxInflection = 0.5 * dt * (v0 + vf);
	
	
	if dx >= dxInflection
		dxVmaxInflection = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * ((2.0 * vmax - v0 - vf) / dt));
		
		if dx < dxVmaxInflection
			% A triangle profile with a peak
			soln.cs = 10;
		else
			% A trapezoid profile with set velocity at the max
			soln.v = vmax;
			soln.a = ((2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / 2.0 - (2.0 * vmax - v0 - vf) * vmax) / (dx - dt * vmax);
			soln.cs = 20;
		end % dxVmaxInflection
		
	else
		dxVminInflection = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * ((v0 + vf - 2.0 * vmin) / dt));
		
		if dx > dxVminInflection
			% A triangle profile with a dip
			soln.cs = 1;
		else
			% A trapezoid profile with set velocity at the min
			soln.a = ((v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / 2.0 - (v0 + vf - 2.0 * vmin) * vmin) / (dx - dt * vmin);
			soln.v = vmin;
			soln.cs = 2;
		end % dxVminInflection
		
	end % dxInflection
	
	if (soln.cs == 10) || (soln.cs == 1)
		p2 = 2.0 * dt;
		p1 = - 4.0 * dx;
		p0 = 2.0 * dx * (v0 + vf) - dt * (v0 ^ 2 + vf ^ 2);
		[solnRoots, validRoots] = Math_2ndOrderRoots(p2,p1,p0);
		if !validRoots
			printf("Kin_GetAcc call invalid: Imaginary roots for case %d\n", soln.cs); valid = false; return;
		else
			if soln.cs == 10
				soln.v = max(solnRoots.r1, solnRoots.r2);
			else
				soln.v = min(solnRoots.r1, solnRoots.r2);
			end
			soln.a = abs(2.0 * soln.v - v0 - vf) / dt;
		end
	end
	
	printf("Kin_GetAcc call: Acc %1.3f, Vel %1.3f, Case %d\n", soln.a, soln.v, soln.cs);
	
end
