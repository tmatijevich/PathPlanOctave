%!octave

dx 		= 0.050:0.030:0.600;
dt 		= [0.225 * ones(size(dx)), 0.225 * ones(size(dx))];
v0 		= [0.900 * ones(size(dx)), 1.425 * ones(size(dx))];
vf 		= [1.600 * ones(size(dx)), 1.279 * ones(size(dx))];
vmin 	= [0.500 * ones(size(dx)), 0.500 * ones(size(dx))];
vmax 	= [2.500 * ones(size(dx)), 2.500 * ones(size(dx))];
a 		= [16.75 * ones(size(dx)), 12.300 * ones(size(dx))];
dx 		= [dx, dx];

for i = 1:length(dx)
	[Solution(i), Valid(i)] = PathVel(dt(i), dx(i), v0(i), vf(i), vmin(i), vmax(i), a(i), true);
end
