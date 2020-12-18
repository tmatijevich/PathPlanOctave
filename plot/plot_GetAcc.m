%!octave

% Set inputs
dt = 0.240; % Time [seconds]
dx = 9.25 * 25.4 * 0.001; % Distance [meters]
v0 = 0.0;
vf = 0.0;
vmin = 0.0;
vmax = 2.5;

% Call the function
[Solution, Valid] = Kin_GetAcc(dt, dx, v0, vf, vmin, vmax);

v = [v0, Solution.v12, Solution.v12, vf];
t = [0.0, Solution.t1, Solution.t2, dt];

% Create the figure
hFig = figure(1, "name", "GetAcc()"); set(hFig, "menubar", "none"); cla;
plot(t, v);
