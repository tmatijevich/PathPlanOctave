%!octave

% Create coefficients
P{1} = [2, 1, 0];
P{2} = [-3.45, 0.02, 10.21];
P{3} = [6, 1, 1];
P{4} = [0, 3, 2];

for i = 1:4
	r = roots(P{i})
	[Solution, Valid] = SecondOrderRoots(P{i}(1), P{i}(2), P{i}(3));
end
