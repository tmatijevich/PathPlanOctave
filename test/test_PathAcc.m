%!octave

% Clear inputs
clear dt_ dx_ v_0_ v_f_ v_min_ v_max_;

% Test input requirements
dt_(1) = 0.1; dx_(1) = 0.1; v_0_(1) = 1.0; v_f_(1) = 1.0; v_min_(1) = 1.0; v_max_(1) = 0.9; % Plausible limits
dt_(2) = 0.1; dx_(2) = 0.1; v_0_(2) = 1.0; v_f_(2) = 1.0; v_min_(2) = 0.5; v_max_(2) = 0.9; % Valid endpoints
dt_(3) = 0.0; dx_(3) = 0.1; v_0_(3) = 1.0; v_f_(3) = 1.0; v_min_(3) = 1.0; v_max_(3) = 0.9; % Positive inputs
dt_(4) = 0.1; dx_(4) = 0.1; v_0_(4) = 1.0; v_f_(4) = 1.0; v_min_(4) = 1.0; v_max_(4) = 2.5; % Move limit
dt_(5) = 0.1; dx_(5) = 0.1; v_0_(5) = 1.0; v_f_(5) = 1.0; v_min_(5) = 0.5; v_max_(5) = 2.5; % Zero acceleration

dt_1 = 0.033:0.040:0.676;
offset = length(dt_);
for i = (offset+1):(offset+length(dt_1))
	dt_(i) = dt_1(i-offset); dx_(i) = 0.311; v_0_(i) = 0.9; v_f_(i) = 0.8; v_min_(i) = 0.5; v_max_(i) = 2.5;
end

dx_1 = 0.053:0.035:0.527;
offset = length(dt_);
for i = (offset+1):(offset+length(dx_1))
	dt_(i) = 0.199; dx_(i) = dx_1(i - offset); v_0_(i) = 1.1; v_f_(i) = 1.3; v_min_(i) = 0.5; v_max_(i) = 2.5;
end

for i = 1:length(dt_)
	printf("%2d: ", i - 1);
	PathAcc(dt_(i), dx_(i), v_0_(i), v_f_(i), v_min_(i), v_max_(i), true);
end
