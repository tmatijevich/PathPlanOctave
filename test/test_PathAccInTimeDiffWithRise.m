%!octave

dt_tilda = 0.005 : 0.010 : 0.285;
dx = 0.275;
v_1 = 2.000;
v_f = 1.500;
v_min = 0.750;
v_max = 3.000;

for i = 1:length(dt_tilda)
	PathAccInTimeDiffWithRise(dt_tilda(i), dx, v_1, v_f, v_min, v_max, true);
end

v_max = 2.25;

for i = 1:length(dt_tilda)
	PathAccInTimeDiffWithRise(dt_tilda(i), dx, v_1, v_f, v_min, v_max, true);
end
