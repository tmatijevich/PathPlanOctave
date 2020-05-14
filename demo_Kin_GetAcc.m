%!octave

dx = 0.25:0.025:0.85; n = length(dx);
dt = 0.300; v0 = 1.9; vf = 1.2;
t = cell(n, 1);
v = cell(n, 1);
a = zeros(n, 1);
for i = 1:n
	[soln, valid] = Kin_GetAcc(dt, dx(i), v0, vf, 0.75, 3.0);
	if valid
		a(i) = soln.a;
		if (soln.cs == 10) || (soln.cs == 1)
			t{i} = [0.0, abs(v0 - soln.v) / soln.a, dt];
			v{i} = [v0, soln.v, vf];
		else
			t{i} = [0.0, abs(v0 - soln.v) / soln.a, dt - abs(vf - soln.v) / soln.a, dt];
			v{i} = [v0, soln.v, soln.v, vf];
		end
	end
end

hFig = figure(1, "name", "Kin_GetAcc()"); set(hFig, "menubar", "none"); cla;
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

hFig = figure(2, "name", "Kin_GetAcc()"); set(hFig, "menubar", "none"); cla;
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
