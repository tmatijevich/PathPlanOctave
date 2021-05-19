%!octave

function [solution, valid] = PathPoint(x_0, t_, v_, n, t, printResult = false)
	% function [solution, valid] = PathPoint(x_0, t_, v_, n, t, printResult = false)
	% Determine the point on a piecewise linear velocity profile
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reset the solution
	solution.x = 0.0;
	solution.v = 0.0;
	solution.a = 0.0;
	
	% Input requirements
	% #1: Number of points
	if (n < 2) || (n > min(length(t_), length(v_)))
		printf("PathPoint call failed: Number of points %d exceeds limits %d, %d\n", n, 2, min(length(t_), length(v_)));
		valid = false;
		return;
	end
		
	% #2: Non-decreasing time points
	for i = 2:n
		if t_(i) < t_(i - 1)
			printf("PathPoint call failed: Time point %d, %.3f is less than point %d, %.3f\n", i, t_(i), i - 1, t_(i-1));
			valid = false;
			return;
		end % Non-decreasing
	end % Loop times
	
	% #3: Valid request time
	if (t < t_(1)) || (t > t_(n))
		printf("PathPoint call failed: Requested time value %.3f exceeds time endpoints %.3f, %.3f\n", t, t_(1), t_(n));
		valid = false;
		return;
	end
	
	% Declare position and acceleration arrays
	x_ = zeros(n, 1);
	a_ = zeros(n, 1);
	
	% Set the initial position
	x_(1) = x_0;
	
	% Loop through each segment
	for i = 2:n
		% Determine position given a piecewise linear velocity profile
		x_(i) = x_(i-1) + 0.5 * (v_(i) + v_(i-1)) * (t_(i) - t_(i-1));
		% Determine the set acceleration for the segment
		if t_(i) == t_(i-1)
			a_(i-1) = 0.0;
		else
			a_(i-1) = (v_(i) - v_(i-1)) / (t_(i) - t_(i-1));
		end
	end % Loop segments
	
	% Find the requested segment
	if t == t_(n)
		seg = n - 1;
	else
		for i = 2:n
			if t < t_(i)
				seg = i - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	% Set solution
	solution.a = a_(seg);
	solution.v = v_(seg) + a_(seg) * (t - t_(seg));
	solution.x = x_(seg) + v_(seg) * (t - t_(seg)) + 0.5 * a_(seg) * (t - t_(seg)) ^ 2;
	valid = true; 
	
	if printResult
		printf("PathPoint call: Pos %.3f, Vel %.3f, Acc %.3f\n", solution.x, solution.v, solution.a);
	end
	
end % Function
