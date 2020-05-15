%!octave

dx = 0.25:0.025:0.85; n = length(dx);
dt = 0.300; v0 = 1.9; vf = 1.2; vmin = 0.75; vmax = 3.0;
% dt = 0.300; v0 = 0.0; vf = 0.0; vmin = 0.0; vmax = 3.0;
t = cell(n, 1);
v = cell(n, 1);
a = zeros(n, 1);
for i = 1:n
	[soln, valid] = Kin_GetAcc(dt, dx(i), v0, vf, vmin, vmax);
	if valid
		a(i) = soln.a;
		t{i} = [0.0, soln.t1, soln.t2, dt];
		v{i} = [v0, soln.v12, soln.v12, vf];
	end
end

hFig = figure(1, "name", "Kin_GetAcc()"); set(hFig, "menubar", "none"); cla;
hold on;
box(gca, "off")
plot(t{4}, v{4}, "r*--");
plot(t{15}, v{15}, "b*--");
plot(t{21}, v{21}, "g*--");
ylim([0 3]);
xlabel("Time [s]");
ylabel("Velocity [m/s]")

hFig = figure(2, "name", "Kin_GetAcc()"); set(hFig, "menubar", "none"); cla;
hold on;
for i = 1:n
	zplt = dx(i) .* ones(1, length(t{i}));
	surf([t{i}(:) t{i}(:)], [v{i}(:) v{i}(:)], [zplt(:) zplt(:)], "facecolor", "none", "edgecolor", "interp", "linewidth", 1.5); 
end
hColorbar = colorbar;
hColorbarTitle = get(hColorbar, "label");
set(hColorbarTitle, "string", "Distance dx [m]");
xlabel("Time [s]");
ylabel("Velocity [m/s]");

hFig = figure(3, "name", "Kin_GetAcc()"); set(hFig, "menubar", "none"); cla;
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
