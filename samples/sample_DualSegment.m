%!octave

t = [0.100, 0.3187, 0.600];
v = [1.200, 1.900, 1.000];
textSize = 13;

hFig = figure(1, "name", "Velocity Dual Segment"); set(hFig, "menubar", "none");
cla;
hold on;
area(t, v, "facecolor", [0.8 0.8 0.8], "edgecolor", "none");
plot(t, v, "b.-", "markersize", 18.0, "linewidth", 1.5);
xlim([0.0 0.7]);
text(t(1) - 0.03, v(1) + 0.1, "v_0, t_0", "fontsize", textSize);
text(t(2)       , v(2) + 0.1, "v_1, t_1", "fontsize", textSize);
text(t(3)       , v(3) + 0.1, "v_f, t_f", "fontsize", textSize);
text(0.350, 0.9, "{\\delta}x", "interpreter", "tex", "fontsize", textSize);
text(0.225, 1.7, "a", "fontsize", textSize);
text(0.500, 1.4, "a", "fontsize", textSize);
axis("nolabel", "tic[]");
box off;
ylabel("Velocity", "fontsize", textSize);
xlabel("Time", "fontsize", textSize);
