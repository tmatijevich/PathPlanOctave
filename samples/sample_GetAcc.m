%!octave

dx = 0.25:0.025:0.85; n = length(dx);
dt = 0.300; v0 = 1.9; vf = 1.2; vmin = 0.75; vmax = 3.0;
% dt = 0.300; v0 = 0.0; vf = 0.0; vmin = 0.0; vmax = 3.0;
t = cell(n, 1);
v = cell(n, 1);
a = zeros(n, 1);
for i = 1:n
	[Solution, Valid] = GetAcc(dt, dx(i), v0, vf, vmin, vmax, true);
	if Valid
		a(i) = Solution.a;
		t{i} = Solution.t;
		v{i} = Solution.v;
	end
end

hFig = figure(1, "name", "GetAcc()"); set(hFig, "menubar", "none"); cla;
hold on;
box(gca, "off")
plot(t{4}, v{4}, "r*--");
plot(t{15}, v{15}, "b*--");
plot(t{21}, v{21}, "g*--");
ylim([0 3]);
TimeString = sprintf("{\\Delta}t = %1.3f s", dt);
DistanceString = sprintf("{\\Delta}x = %1.3f, %1.3f, %1.3f m", dx(4), dx(15), dx(21));
VelocityString = sprintf("v_{min} = %1.3f {\\leq} v_0 = %1.3f, v_f = %1.3f {\\leq} v_{max} = %1.3f m/s", vmin, v0, vf, vmax);
TitleString = [TimeString, "\n" , DistanceString, "\n", VelocityString];
title(TitleString, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]")

hFig = figure(2, "name", "GetAcc()"); set(hFig, "menubar", "none"); cla;
hold on;
for i = 1:n
	zplt = dx(i) .* ones(1, length(t{i}));
	surf([t{i}(:) t{i}(:)], [v{i}(:) v{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5); 
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", 'Distance {\delta}x [m]');
set(hColorbarTitle, "interpreter", "tex");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

hFig = figure(3, "name", "GetAcc()"); set(hFig, "menubar", "none"); cla;
hold on;
for i = 1:n
	zplt = a(i) .* ones(1, length(t{i}));
	surf([t{i}(:) t{i}(:)], [v{i}(:) v{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5); 
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Acceleration a [m/s^2]");
xlabel("Time [s]");
ylabel("Velocity [m/s]");
