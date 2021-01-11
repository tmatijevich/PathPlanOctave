%!octave

% GetAcc() sample

% Set velocities
v0 		= 1.9;
vf 		= 1.2;
vmin 	= 0.75;
vmax 	= 3.0;

% Create the figure
hFig = figure(1, "name", "GetAcc()"); set(hFig, "menubar", "none");
CurrentPosition = get(gcf, "position");
set(gcf, "position", [CurrentPosition(1:2) 860 720]);

%===============================================================================
% Fix time, distance range
dt = 0.300;
dx_ = 0.25:0.025:0.85;
n 	= length(dx_);

ct = cell(n,1);
cv = cell(n,1);
a_ = zeros(n,1);

for i = 1:n
	[Solution, Valid] = GetAcc(dt, dx_(i), v0, vf, vmin, vmax, true);
	if Valid
		ct{i} = Solution.t;
		cv{i} = Solution.v;
		a_(i) = Solution.a;
	end
end

% Create the first subplot
subplot(3,2,1);
cla;
hold on;
title("GetAcc({\\delta}t = 0.300 s, ...)", "interpreter", "tex");
for i = 1:n
	zplt = dx_(i) .* ones(1, length(ct{i}));
	surf([ct{i}(:) ct{i}(:)], [cv{i}(:) cv{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5);
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Distance {\\delta}x [m]");
set(hColorbarTitle, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

% Create the second subplot
subplot(3,2,3);
cla;
hold on;
title("GetAcc({\\delta}t = 0.300 s, ...)", "interpreter", "tex");
for i = 1:n
	zplt = a_(i) .* ones(1, length(ct{i}));
	surf([ct{i}(:) ct{i}(:)], [cv{i}(:) cv{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5);
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Acceleration a [m/s^2]");
% set(hColorbarTitle, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

% Create the third scatter plot
subplot(3,2,5);
cla;
hold on;
title("GetAcc() vs Distance");
NominalDistance = 0.5 * (v0 + vf) * dt;
plot([NominalDistance, NominalDistance], [0, 50], "k--");
plot(dx_, a_, "r.", "markersize", 12);
text(NominalDistance + 0.025, 15, "ACC\\_DEC", "rotation", 90);
text(NominalDistance - 0.025, 15, "DEC\\_ACC", "rotation", 90);
VmaxDistance = (2.0 * vmax ^ 2 - v0 ^ 2 - vf ^ 2) / (2.0 * ((2.0 * vmax - v0 - vf) / dt));
VminDistance = (v0 ^ 2 + vf ^ 2 - 2.0 * vmin ^ 2) / (2.0 * ((v0 + vf - 2.0 * vmin) / dt));
plot([VmaxDistance, VmaxDistance], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
plot([VminDistance, VminDistance], [0, 50], "color", [0.5 0.5 0.5], "linestyle", "--");
text(VmaxDistance + 0.025, 45, "Saturated");
text(VminDistance - 0.125, 45, "Saturated");
ylabel("Acceleration a [m/s^2]");
xlabel("Distance {\\delta}x [m]", "interpreter", "tex");
