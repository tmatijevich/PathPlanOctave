%!octave

function [soln, valid] = Kin_GetVelProfPoint(x0, timePoints, velPoints, n, t)
	% Determine the point on a piecewise linear velocity profile
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Assume the result will be valid
	valid = true;
	soln.x = 0.0; soln.v = 0.0; soln.a = 0.0; % Fallback/invalid solution
	
	N = 20;
	accPoints = zeros(N,1);
	posPoints = zeros(N,1);
	
	% Condition 1: Number of points
	if (n < 2) || (n > N)
		printf("Kin_GetVelProfPoint call invalid: Invalid number of points %d\n", n); valid =  false; return;
	end
	
	% Condition #2: Sequential times
	for i = 2:n
		if timePoints(i) < timePoints(i - 1)
			printf("Kin_GetVelProfPoint call invalid: Non-sequential time points %1.3f, %1.3f\n", timePoints(i - 1), timePoints(i)); valid =  false; return;
		end
	end
	
	% Condition #3: Request time
	if (t < timePoints(1)) || (t > timePoints(n))
		printf("Kin_GetVelProfPoint call invalid: Request time %1.3f exceeds limits %1.3f, %1.3f\n", t, timePoints(1), timePoints(n)); valid =  false; return;
	end
	
	% Compute starting position and acceleration for each segment
	posPoints(1) = x0;
	for i = 2:n
		posPoints(i) = posPoints(i - 1) + 0.5 * (velPoints(i) + velPoints(i - 1)) * (timePoints(i) - timePoints(i - 1));
		accPoints(i) = (velPoints(i) - velPoints(i - 1)) / (timePoints(i) - timePoints(i - 1));
	end
	
	% Find the requested segment
	if (t == timePoints(n))
		segment = n - 1;
	else
		for i = 2:n
			if t < timePoints(i)
				segment = i - 1; break;
			end
		end
	end
	
	soln.a = accPoints(segment + 1);
	soln.v = velPoints(segment) + soln.a * (t - timePoints(segment));
	soln.x = posPoints(segment) + velPoints(segment) * (t - timePoints(segment)) + 0.5 * soln.a * (t - timePoints(segment)) ^ 2;
	
	printf("Kin_GetVelProfPoint call: Pos %1.3f, Vel %1.3f, Acc %1.3f\n", soln.x, soln.v, soln.a);
	
end
