%!octave

function [] = plot_GetAccInTimeDiff(tdiff, dx, v0, vf, vmin, vmax)
	[Solution, Valid] = GetAccInTimeDiff(tdiff, dx, v0, vf, vmin, vmax, true);
	if !Valid
		return;
	else
		hFig = figure(1, "name", "GetAccInTimeDiff()");
		set(hFig, "menubar", "none");
		cla;
		
		hold on
		plot(Solution.vmin.t, Solution.vmin.v, "g");
		plot(Solution.vmax.t, Solution.vmax.v, "b");
	end
end
