%!octave

clear In;

% Create inputs
In{1}.dt = 0.100; In{1}.dx = 0.100; In{1}.v0 = 1.000; In{1}.vf = 1.000; In{1}.vmin = 0.500; In{1}.vmax = 2.500;
In{2}.dt = 0.100; In{2}.dx = 0.100; In{2}.v0 = 1.000; In{2}.vf = 1.000; In{2}.vmin = 1.000; In{2}.vmax = 2.500;
In{3}.dt = 0.000; In{3}.dx = 0.100; In{3}.v0 = 1.000; In{3}.vf = 1.000; In{3}.vmin = 0.500; In{3}.vmax = 2.500;
In{4}.dt = 0.000; In{4}.dx = 0.100; In{4}.v0 = 1.000; In{4}.vf = 1.000; In{4}.vmin = 0.500; In{4}.vmax = 0.900;
In{5}.dt = 0.000; In{5}.dx = 0.100; In{5}.v0 = 1.000; In{5}.vf = 1.000; In{5}.vmin = 1.000; In{5}.vmax = 0.900;
dt_ = 0.033:0.040:0.676; % 17
offset = length(In);
for i = (offset + 1):(offset + length(dt_))
	In{i}.dt = dt_(i-offset); In{i}.dx = 0.311; In{i}.v0 = 0.900; In{i}.vf = 0.800; In{i}.vmin = 0.500; In{i}.vmax = 2.500;
end
dx_ = 0.053:0.035:0.527; % 14
offset = length(In);
for i = (offset + 1):(offset + length(dx_))
	In{i}.dt = 0.199; In{i}.dx = dx_(i - offset); In{i}.v0 = 1.100; In{i}.vf = 1.300; In{i}.vmin = 0.500; In{i}.vmax = 2.500;
end

for i = 1:length(In) %36
	printf("%d: ", i-1);
	[Solution, Valid] = PathAcc(In{i}.dt, In{i}.dx, In{i}.v0, In{i}.vf, In{i}.vmin, In{i}.vmax, true);
end
