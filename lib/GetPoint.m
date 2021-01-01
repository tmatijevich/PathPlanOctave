%!octave

function [Solution, Valid] = GetPoint(x0, TimePoints, VelocityPoints, NumberOfPoints, t, PrintResult = false)
	% Determine the point on a piecewise linear velocity profile
	% Date: 2020-04-01
	% Created by: Tyler Matijevich
	
	% Reset the solution
	Solution.x = 0.0;
	Solution.v = 0.0;
	Solution.a = 0.0;
	
	% Input requirements
	% #1: Number of points
	if (NumberOfPoints < 2) || (NumberOfPoints > min(length(TimePoints), length(VelocityPoints)))
		printf("GetPoint call failed: Number of points %d exceeds limits %d, %d\n", NumberOfPoints, 2, min(length(TimePoints), length(VelocityPoints)));
		Valid = false;
		return;
	end
		
	% #2: Sequential times
	for i = 2:NumberOfPoints
		if TimePoints(i) < TimePoints(i - 1)
			printf("GetPoint call failed: Time point %d, %.3f is less than point %d, %.3f\n", i, TimePoints(i), i - 1, TimePoints(i-1));
			Valid = false;
			return;
		end % Higher index, less time?
	end % For loop time array
	
	% #3: Valid request time
	if (t < TimePoints(1)) || (t > TimePoints(NumberOfPoints))
		printf("GetPoint call failed: Requested time value %.3f exceeds time endpoints %.3f, %.3f\n", t, TimePoints(1), TimePoints(NumberOfPoints));
		Valid = false;
		return;
	end
	
	% Compute starting position and acceleration for each segment
	x = zeros(length(TimePoints), 1);
	v = VelocityPoints; % Copy velocity array
	a = zeros(length(TimePoints), 1);
	x(1) = x0;
	for i = 2:NumberOfPoints
		% Position given a linear velocity profile
		x(i) = x(i-1) + 0.5 * (v(i) + v(i-1)) * (TimePoints(i) - TimePoints(i-1));
		% Constant acceleration - defined for a segment
		if TimePoints(i) == TimePoints(i-1)
			a(i-1) = 0.0;
		else
			a(i-1) = (v(i) - v(i-1)) / (TimePoints(i) - TimePoints(i-1));
		end
	end
	
	% Find the requested segment
	if t == TimePoints(NumberOfPoints)
		Segment = NumberOfPoints - 1;
	else
		for i = 2:NumberOfPoints
			if t < TimePoints(i)
				Segment = i - 1;
				break;
			end % Within time?
		end % Loop array
	end % Final segment?
	
	% Set solution
	Solution.a = a(Segment);
	Solution.v = v(Segment) + a(Segment) * (t - TimePoints(Segment));
	Solution.x = x(Segment) + v(Segment) * (t - TimePoints(Segment)) + 0.5 * a(Segment) * (t - TimePoints(Segment)) ^ 2;
	Valid = true; 
	
	if PrintResult
		printf("GetPoint call: Pos %.3f, Vel %.3f, Acc %.3f\n", Solution.x, Solution.v, Solution.a);
	end
	
end % Function
