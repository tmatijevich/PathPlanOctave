%!octave

clear dt v_0 v_f v_min v_max a;

dt(1) = 0.100; v_0(1) = 1.000; v_f(1) = 1.000; v_min(1) = 0.500; v_max(1) = 2.500; a(1) = 10.000;
dt(2) = 0.100; v_0(2) = 1.000; v_f(2) = 1.000; v_min(2) = 0.500; v_max(2) = 0.400; a(2) = 10.000;
dt(3) = 0.100; v_0(3) = 1.000; v_f(3) = 2.600; v_min(3) = 0.500; v_max(3) = 2.500; a(3) = 10.000;
dt(4) = 0.100; v_0(4) = 1.000; v_f(4) = 1.000; v_min(4) = 0.500; v_max(4) = 2.500; a(4) =  0.000;
dt(5) = 0.100; v_0(5) = 1.000; v_f(5) = 2.000; v_min(5) = 0.500; v_max(5) = 2.500; a(5) =  1.000;

dt_range = 0.015:0.030:0.550;
i_1 = length(dt) + 1;
for i = i_1:(i_1+length(dt_range)-1)
	dt(i) = dt_range(i - i_1 + 1);
	v_0(i) = 0.800;
	v_f(i) = 1.200;
	v_min(i) = 0.500;
	v_max(i) = 2.500;
	a(i) = 7.500;
end

a_range = 0.1:0.8:9.0;
i_2 = length(dt) + 1;
for i = i_2:(i_2+length(a_range)-1)
	dt(i) = 0.300;
	v_0(i) = 0.800;
	v_f(i) = 1.200;
	v_min(i) = 0.500;
	v_max(i) = 2.500;
	a(i) = a_range(i - i_2 + 1);
end

printf("=== Test input validation ===\n");
for i = 1:length(dt)
	if (i == i_1) 
		printf("=== Vary input time duration ===\n");
	elseif (i == i_2) 
		printf("=== Vary input acceleration ===\n");
	end
	printf("%2d: ", i-1);
	PathDist(dt(i), v_0(i), v_f(i), v_min(i), v_max(i), a(i), true);
end
