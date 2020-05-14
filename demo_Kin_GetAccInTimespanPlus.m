%!octave

tdiff = 0.02:0.01:0.280; n = length(tdiff);
v1 = 1.9; vf = 1.2; vmin = 0.75; vmax = 3.0;
dx = 0.350;
t = cell(n,1);
v = cell(n,1);
a = zeros(1,n);

for i = 1:n
	[soln, valid] = Kin_GetAccInTimespanPlus(tdiff(i), dx, v1, vf, vmin, vmax);
	if valid
		%
	end
end
