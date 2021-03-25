%!octave

% Create coefficients
P{1} = [2, 1, 0];
P{2} = [-3.45, 0.02, 10.21];
P{3} = [6, 1, 1];
P{4} = [0, 3, 2];
P{5} = [0, 0, 2];
P{6} = [0, 0, 0];
P{7} = [1, 2, 1];
P{8} = [0.2, -0.4, 0.2];

for i = 1:length(P)
	printf("Coefficients: %.3f, %.3f, %.3f\n", P{i}(1), P{i}(2), P{i}(3));
	r = roots(P{i});
	switch(length(r))
		case 0 printf("Octave roots call: No solution\n");
		case 1 printf("Octave roots call: %.3f\n", r);
	 	case 2 printf("Octave roots call: %.3f%+.3fj, %.3f%+.3fj\n", real(r(1)), imag(r(1)), real(r(2)),  imag(r(2)));
	end
	[Solution, Valid] = SecondOrderRoots(P{i}(1), P{i}(2), P{i}(3), true);
	printf("\n");
end
