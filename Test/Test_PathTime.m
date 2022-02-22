%!octave

clear dx_ v_0_ v_f_ v_min_ v_max_ a_;

% Test input requirements

dx_(1) = 0.1; v_0_(1) = 1.0; v_f_(1) = 1.0; v_min_(1) = 1.0; v_max_(1) = 0.9; a_(1) = 10.0; % Plausible limits
dx_(2) = 0.1; v_0_(2) = 1.0; v_f_(2) = 1.0; v_min_(2) = 0.5; v_max_(2) = 0.9; a_(2) = 10.0; % Valid endpoints
dx_(3) = 0.0; v_0_(3) = 1.0; v_f_(3) = 1.0; v_min_(3) = 0.5; v_max_(3) = 2.5; a_(3) = 10.0; % Positive inputs
dx_(4) = 0.1; v_0_(4) = 2.0; v_f_(4) = 1.0; v_min_(4) = 1.0; v_max_(4) = 2.5; a_(4) = 10.0; % Move limit

dx_1 = 0.050:0.265:3.500;
offset = length(dx_);
for i = (offset + 1):(offset+length(dx_1))
	dx_(i) = dx_1(i-offset); v_0_(i) = 0.75; v_f_(i) = 0.65; v_min_(i) = 0.5; v_max_(i) = 2.5; a_(i) = 10.0;
end

a_1 = 0.1:0.8:9.0;
offset = length(dx_);
for i = (offset + 1):(offset+length(a_1))
	dx_(i) = 0.300; v_0_(i) = 0.8; v_f_(i) = 1.2; v_min_(i) = 0.5; v_max_(i) = 2.5; a_(i) = a_1(i - offset);
end

printf("=== Test input requirements ===\n");
for i = 1:length(dx_)
	if i == length(dx_) - length(a_1) - length(dx_1) + 1
		printf("=== Vary input distance ===\n");
	elseif i == length(dx_) - length(a_1) + 1
		printf("=== Vary input acceleration ===\n");
	end
	printf("%2d: ", i - 1);
	PathTime(dx_(i), v_0_(i), v_f_(i), v_min_(i), v_max_(i), a_(i), true);
end
