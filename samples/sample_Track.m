%!octave

t_s = 0.005;
tPlot = 0.0:t_s:1.5;
n = length(tPlot);
xtPlot = zeros(size(tPlot));
vtPlot = zeros(size(tPlot));
xPlot = zeros(size(tPlot));
vPlot = zeros(size(tPlot));
aPlot = zeros(size(tPlot));

xtPlot(1) = 250.0; % 250 mm away
vtPlot(1) = 1500.0;

for idx = 2:n
    if tPlot(idx) <= 0.250 
        xtPlot(idx) = xtPlot(idx - 1) + vtPlot(1) * t_s; % Moving at 1500 mm/s
        vtPlot(idx) = vtPlot(1);
    elseif tPlot(idx) > 0.75 && tPlot(idx) < 1.25
        xtPlot(idx) = xtPlot(idx - 1) - vtPlot(1) * t_s; % Moving at 1500 mm/s
        vtPlot(idx) = -1.0 * vtPlot(1);
    else
        xtPlot(idx) = xtPlot(idx - 1);
        vtPlot(idx) = 0.0;
    end

    printf("%3d %1.3f s ", idx, tPlot(idx));

    [trackSolution, trackValid] = PathTrack(xPlot(idx - 1), vPlot(idx - 1), t_s, xtPlot(idx), vtPlot(idx), 2500.0, 15000.0, true);
    if trackValid
        xPlot(idx) = trackSolution.x;
        vPlot(idx) = trackSolution.v;
        aPlot(idx) = trackSolution.a;
    else
        return;
    end

end