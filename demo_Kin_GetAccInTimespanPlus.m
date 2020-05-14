%!octave

tdiff = 0.02:0.01:0.280; n = length(tdiff);
v1 = 1.9; vf = 1.2; vmin = 0.75; vmax = 3.0;
dx = 0.350;
t = cell(n,1);
v = cell(n,1);
a = zeros(n, 1);

for i = 1:n
	[soln, valid] = Kin_GetAccInTimespanPlus(tdiff(i), dx, v1, vf, vmin, vmax);
	if valid
		tr = v1 / soln.a;
		dxr = (v1 ^ 2) / (2.0 * soln.a);
		[solnTimes, validTimes] = Kin_GetTimespan(dx - dxr, v1, vf, vmin, vmax, soln.a);
		if validTimes
			
			% vmin points
			vtemp = [v1, solnTimes.v1min, solnTimes.v1min, vf];
			ttemp = tr + [0.0, solnTimes.tVmin1, solnTimes.tVmin2, solnTimes.tVmin];
			vtemp = [fliplr(vtemp), 0.0, v1];
			ttemp = [fliplr(ttemp), 0.0, tr];
			
			% vmax points
			vtemp = [vtemp, solnTimes.v1max, solnTimes.v1max, vf];
			ttempMAX = tr + [solnTimes.tVmax1, solnTimes.tVmax2, solnTimes.tVmax];
			ttemp = [ttemp, ttempMAX];
			
			% all together now
			v{i} = vtemp;
			t{i} = ttemp;
			a(i) = soln.a;
		end
	end
end


hFig = figure(1, "name", "Kin_GetAccInTimespanPlus()"); set(hFig, "menubar", "none"); cla;
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
