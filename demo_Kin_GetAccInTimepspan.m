%!octave

tdiff = 0.02:0.01:0.400; n = length(tdiff);
v0 = 1.9; vf = 1.2; vmin = 0.75; vmax = 3.0;
dx = 0.350;

% hFig = figure(1, "name", "Kin_GetAccInTimespan()"); set(hFig, "menubar", "none"); cla;
% hold on;
for i = 1:n
	[soln, valid] = GetAccInTimeDiff(tdiff(i), dx, v0, vf, vmin, vmax, true);
end
