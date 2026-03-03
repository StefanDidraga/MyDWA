function handler = plot_obs(pose, r, color)
%Plots moving obstacles

    x = pose(1);
    y = pose(2);

    aplha = 0:pi/40:2*pi;
    x = x + r * cos(aplha);
    y = y + r * sin(aplha);
    circle = plot(x, y, 'LineWidth', 1.2, 'LineStyle', '-', 'color', color);

    handler = circle;

end