%!octave

% Distance range
dx_ = 0.25:0.025:0.85;
n 	= length(dx_);

% Set velocities
v0 		= 1.9;
vf 		= 1.2;
vmin 	= 0.75;
vmax 	= 3.0;

% GetAcc(), fix time
%===============================================================================
dt = 0.300;

Acc.t = cell(n,1);
Acc.v = cell(n,1);
Acc.a = zeros(n,1);

for i = 1:n
	[Solution, Valid] = GetAcc(dt, dx_(i), v0, vf, vmin, vmax, true);
	if Valid
		Acc.t{i} = Solution.t;
		Acc.v{i} = Solution.v;
		Acc.a(i) = Solution.a;
	end
end

hFig = figure(1, "name", "Varying Distance"); set(hFig, "menubar", "none");

subplot(2,2,1);
cla;
hold on;
box(gca, "off");
title("GetAcc({\\delta}t = 0.300 s)", "interpreter", "tex");
for i = 1:n
	zplt = Acc.a(i) .* ones(1, length(Acc.t{i}));
	surf([Acc.t{i}(:) Acc.t{i}(:)], [Acc.v{i}(:) Acc.v{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5);
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", 'Acceleration a [m/s^2]');
% set(hColorbarTitle, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

%===============================================================================
subplot(2,2,3)
cla;
hold on;
box(gca, "off");
title("GetAcc() vs Distance");
plot(dx_, Acc.a, "r.", "markersize", 12);
ylabel("Acceleration a [m/s^2]");
xlabel("Distance {\\delta}x [m]", "interpreter", "tex");

% GetTime(), fix acceleration
% ==============================================================================
dx = 0.450;

dt_ = 0.170:0.010:0.500;
n 	= length(dt_);

Acc.t = cell(n,1);
Acc.v = cell(n,1);
Acc.a = zeros(n,1);

for i = 1:n
	[Solution, Valid] = GetAcc(dt_(i), dx, v0, vf, vmin, vmax, true);
	if Valid
		Acc.t{i} = Solution.t;
		Acc.v{i} = Solution.v;
		Acc.a(i) = Solution.a;
	end
end

subplot(2,2,2)
cla;
hold on;
box(gca, "off");
title("GetAcc({\\delta}x = 0.450 m)", "interpreter", "tex");
for i = 1:n
	zplt = Acc.a(i) .* ones(1, length(Acc.t{i}));
	surf([Acc.t{i}(:) Acc.t{i}(:)], [Acc.v{i}(:) Acc.v{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5);
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", 'Acceleration a [m/s^2]');
% set(hColorbarTitle, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

%===============================================================================
subplot(2,2,4)
cla;
hold on;
box(gca, "off");
title("GetAcc() vs Time Duration");
plot(dt_, Acc.a, "b.", "markersize", 12);
ylabel("Acceleration a [m/s^2]");
xlabel("Time Duration {\\delta}t [s]", "interpreter", "tex");
