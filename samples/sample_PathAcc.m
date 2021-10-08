%!octave

% PathAcc() sample

% Set velocities
v0 		= 1.9;
vf 		= 1.2;
vmin 	= 0.75;
vmax 	= 3.0;

% Create the figure
hFig = figure(1, "name", "PathAcc() Sample"); set(hFig, "menubar", "none");
CurrentPosition = get(gcf, "position");
set(gcf, "position", [CurrentPosition(1:2) 800 600]);
textSize = 13;
tickSize = 10;

%===============================================================================
% Fix time, distance range
dt = 0.300;
dx_ = 0.25:0.025:0.85;
n 	= length(dx_);

ct = cell(n,1);
cv = cell(n,1);
a_ = zeros(n,1);

for i = 1:n
	[Solution, Valid] = PathAcc(dt, dx_(i), v0, vf, vmin, vmax, true);
	if Valid
		ct{i} = Solution.t_;
		cv{i} = Solution.v_;
		a_(i) = Solution.a;
	end
end

% Create the second subplot
subplot(2,2,1);
cla;
hold on;
title("PathAcc({\\delta}t = 0.300 s, ...)", "interpreter", "tex", "fontsize", textSize);
for i = 1:n
	zplt = a_(i) .* ones(1, length(ct{i}));
	surf([ct{i}(:) ct{i}(:)], [cv{i}(:) cv{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5);
end
hColorbar = colorbar;
set(hColorbar, "fontsize", tickSize);
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Acceleration a [m/s^2]", "fontsize", textSize);
% set(hColorbarTitle, "interpreter", "tex");
set(gca, "fontsize", tickSize);
xlabel("Time [s]", "fontsize", textSize);
ylabel("Velocity [m/s]", "fontsize", textSize);

% Create the third scatter plot
subplot(2,2,3);
cla;
hold on;
title("PathAcc() vs Distance", "fontsize", textSize);
NominalDistance = 0.5 * (v0 + vf) * dt;
plot([NominalDistance, NominalDistance], [0, 50], "k--");
plot(dx_, a_, "r.", "markersize", 12);
text(NominalDistance + 0.028, 15, "ACC\\_DEC", "rotation", 90);%, "fontsize", textSize);
text(NominalDistance - 0.028, 15, "DEC\\_ACC", "rotation", 90);%, "fontsize", textSize);
VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * ((2.0 * vmax - v0 - vf) / dt));
VminDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * ((v0 + vf - 2.0 * vmin) / dt));
plot([VmaxDistance, VmaxDistance], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
plot([VminDistance, VminDistance], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
text(VmaxDistance + 0.020, 45, "Saturated");%, "fontsize", textSize);
text(VminDistance - 0.140, 45, "Saturated");%, "fontsize", textSize);
set(gca, "fontsize", tickSize);
ylabel("Acceleration a [m/s^2]", "fontsize", textSize);
xlabel("Distance {\\delta}x [m]", "interpreter", "tex", "fontsize", textSize);

%===============================================================================
% Fix distance, time range
dx = 0.450;
dt_ = 0.170:0.010:0.500;
n 	= length(dt_);

ct = cell(n,1);
cv = cell(n,1);
a_ = zeros(n,1);

for i = 1:n
	[Solution, Valid] = PathAcc(dt_(i), dx, v0, vf, vmin, vmax, true);
	if Valid
		ct{i} = Solution.t_;
		cv{i} = Solution.v_;
		a_(i) = Solution.a;
	end
end

% Create the second subplot
subplot(2,2,2);
cla;
hold on;
title("PathAcc(..., {\\delta}x = 0.450 m, ...)", "interpreter", "tex", "fontsize", textSize);
for i = 1:n
	zplt = a_(i) .* ones(1, length(ct{i}));
	surf([ct{i}(:) ct{i}(:)], [cv{i}(:) cv{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.2);
end
hColorbar = colorbar;
set(hColorbar, "fontsize", tickSize);
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Acceleration a [m/s^2]", "fontsize", textSize);
% set(hColorbarTitle, "interpreter", "tex");
set(gca, "fontsize", tickSize);
xlabel("Time [s]", "fontsize", textSize);
ylabel("Velocity [m/s]", "fontsize", textSize);

% Create the third scatter plot
subplot(2,2,4);
cla;
hold on;
title("PathAcc() vs Time Duration", "fontsize", textSize);
NominalTime = (2.0 * dx * abs(v0 - vf)) / (abs(v0 ^ 2 - vf ^ 2));
plot([NominalTime, NominalTime], [0, 50], "k--");
plot(dt_, a_, "b.", "markersize", 12);
text(NominalTime + 0.015, 15, "ACC\\_DEC", "rotation", 90);%, "fontsize", textSize);
text(NominalTime - 0.015, 15, "DEC\\_ACC", "rotation", 90);%, "fontsize", textSize);
VmaxTime = (2.0 * dx * abs(2.0 * vmax - v0 - vf)) / (abs(2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2));
VminTime = (2.0 * dx * abs(v0 + vf - 2.0 * vmin)) / (abs(v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2));
plot([VmaxTime, VmaxTime], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
plot([VminTime, VminTime], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
text(VmaxTime - 0.030, 45, "Sat.");%, "fontsize", textSize);
text(VminTime + 0.010, 45, "Saturated");%, "fontsize", textSize);
% xlim([0.0 0.5]);
set(gca, "fontsize", tickSize);
ylabel("Acceleration a [m/s^2]", "fontsize", textSize);
xlabel("Time Duration {\\delta}t [s]", "interpreter", "tex", "fontsize", textSize);
