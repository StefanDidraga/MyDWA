clear all;
clc;

addpath("MotionPlanning\MyDWA\MyStuff\");
addpath("MotionPlanning\MyDWA\utils\data\map\");



%% load environment
% load("gridmap_20x30_empty.mat");
% [m, ~] = size(grid_map);
%     obs_index = find(grid_map==2);
%     obs = [mod(obs_index - 1, m) + 1, fix((obs_index - 1) / m) + 1];
% 
%     obs = [obs ;[6, 6]; [6, 12]; [6, 18]; [12, 6]; [12, 12]; [12, 18]];
%     for i = 10:0.1:30
%         obs = [obs; [14, i]];
%     end
% 
%     for i = 5:0.1:15
%         obs = [obs; [i, 25]];
%     end
%% environment to comp with c++

%obs=[[5,5];[4,6];[5,6]];
obs=[];
%% start and goal pose
start = [5, 0, 0];
goal = [5, 10];

robot.x = start(1);
robot.y = start(2);
robot.theta = start(3);
robot.vx = 0;
robot.w = 0;

robot2.x = start(1);
robot2.y = start(2);
robot2.theta = start(3);
robot2.vx = 0;
robot2.w = 0;

% threshold
maxtime = 20;
dt = 0.1;
max_dist = 0.2;



%% test to comp wit c++

 % [v_xstar , wstar] = my_dwa_nopath(robot, goal, obs);
 % 
 % fprintf("Command velocity: vx = %f4.2, w = %f4.2", v_xstar, wstar);

%% main loop
% sim_time = 0;
% 
% robotPlot = [];
% distoobs = [];
% 
% 
% while sim_time < maxtime
%     [v_xstar , wstar] = my_dwa(robot, goal, obs);
%     robot = f(robot, [v_xstar , wstar], dt);
%     sim_time = sim_time + dt;
%     if dist([robot.x, robot.y],goal')< max_dist
%         fprintf('Goal reached! \n');
%         break
%     end
%     robotPlot = [robotPlot; robot.x, robot.y, robot.theta];
%     distoobs = [distoobs; min(dist(obs, [robot.x; robot.y]))];
% end
% 
% if sim_time > maxtime
%     fprintf('Goal was not reached in the given time \n')
% else
%     fprintf('goal reached in %4.2f', sim_time)
% end
%% main loop nopath
sim_time = 0;

robotPlot2 = [];
distoobs2 = [];


while sim_time < maxtime
    [v_xstar , wstar] = my_dwa_nopath(robot2, goal, obs);
    robot2 = f(robot2, [v_xstar , wstar], dt);
    sim_time = sim_time + dt;
    if dist([robot2.x, robot2.y],goal')< max_dist
        fprintf('Goal reached! \n');
        break
    end
    robotPlot2 = [robotPlot2; robot2.x, robot2.y, robot2.theta];
    distoobs2 = [distoobs2; min(dist(obs, [robot2.x; robot2.y]))];
end

if sim_time > maxtime
    fprintf('Goal was not reached in the given time \n')
else
    fprintf('goal reached in %4.2f', sim_time)
end

%% error calculations

datacpp = readmatrix("DWA_traj_Cpp_tightAngle.csv");

lc = length(datacpp);
lm = length(robotPlot2);
ldata = (1:min(lm,lc));
ltime = max(lm,lc);
zeroVector = (zeros(abs(lm-lc),1))';

Cx = datacpp(1,ldata);
Cy = datacpp(2,ldata);
Ctheta = datacpp(3,ldata);

Mx = robotPlot2(ldata,1);
My = robotPlot2(ldata,2);
Mtheta = robotPlot2(ldata,3);

errorX = Mx' - Cx;
errorX = [errorX, zeroVector];
errorY = My' - Cy;
errorY = [errorY, zeroVector];
errorTheta = Mtheta' - Ctheta;
errorTheta = [errorTheta, zeroVector];

timeVector = (1:ltime) * 0.1;


%% plot


% Create a single, new figure window
figure('Position', [100, 100, 800, 600]);

% --- TOP BIG PLOT (Taller) ---
% 3 rows, 3 columns. Span positions 1 through 6 (the entire top two rows)
subplot(3, 3, [1:6]); 
hold on;
plot(robotPlot2(:,1), robotPlot2(:,2), 'g--', 'LineWidth', 2);
plot(datacpp(1,:), datacpp(2,:), 'b');
scatter(goal(1), goal(2), 100, 'red', 'filled', 'Marker', 'o');
if(~isempty(obs))
scatter(obs(:,1), obs(:,2), 15, 'black', 'filled', 'square');
end
legend("MATLAB", "C++", "Goal", "Obstacles");
title("Robot Trajectory Comparison");
hold off;

% --- BOTTOM LEFT: X Error (Shorter) ---
% Position 7 (first slot of the 3rd row)
subplot(3, 3, 7);
plot(timeVector, errorX);
xlabel("time [s]");
ylabel("error [m]");
title("X error");

% --- BOTTOM MIDDLE: Y Error (Shorter) ---
% Position 8 (second slot of the 3rd row)
subplot(3, 3, 8);
plot(timeVector, errorY);
xlabel("time [s]");
ylabel("error [m]");
title("Y error");

% --- BOTTOM RIGHT: Heading Error (Shorter) ---
% Position 9 (third slot of the 3rd row)
subplot(3, 3, 9);
plot(timeVector, errorTheta);
xlabel("time [s]");
ylabel("error [rad]");
title("Heading error");
%% heading stuff

hdata1 = readmatrix("DWA_heading_Cpp.csv");
hdata2 = readmatrix("DWA_heading_Cpp_noWrap.csv");

figure('Name', 'DWA Heading Comparison: Wrap vs No-Wrap', 'Color', 'w');
tiledlayout(2,2);

% --- Graph 1: Angle to Goal ---
nexttile;
plot(hdata1(1,:), 'LineWidth', 1.5, 'DisplayName', 'With Wrap'); hold on;
plot(hdata2(1,:), '--', 'LineWidth', 1.5, 'DisplayName', 'No Wrap');
title('Angle to Goal (Degrees)');
xlabel('Simulation Step'); ylabel('Degrees');
legend; grid on;

% --- Graph 2: Robot Heading ---
nexttile;
plot(hdata1(2,:), 'LineWidth', 1.5); hold on;
plot(hdata2(2,:), '--', 'LineWidth', 1.5);
title('Robot Heading (Theta)');
xlabel('Simulation Step'); ylabel('Degrees');
grid on;

% --- Graph 3: Angular Difference ---
nexttile;
plot(hdata1(3,:), 'LineWidth', 1.5, 'Color', [0 0.5 0]); hold on;
plot(hdata2(3,:), '--', 'LineWidth', 1.5, 'Color', [0.8 0 0]);
title('Difference (Goal - Heading)');
xlabel('Simulation Step'); ylabel('Degrees');
subtitle('Critical for Steering Logic');
grid on;

% --- Graph 4: Heading Weight (Score) ---
nexttile;
plot(hdata1(4,:), 'LineWidth', 1.5, 'Color', 'm'); hold on;
plot(hdata2(4,:), '--', 'LineWidth', 1.5, 'Color', 'k');
title('Final Heading Weight');
xlabel('Simulation Step'); ylabel('Weight Value');
subtitle('Higher is Better');
grid on;

%% func

% Robot Kinematics
function robot = f(robot, u, dt)
%@breif: robotic kinematic
    F = [ 1 0 0 0 0
          0 1 0 0 0
          0 0 1 0 0
          0 0 0 0 0
          0 0 0 0 0];
 
    B = [dt * cos(robot.theta) 0
            dt * sin(robot.theta)  0
            0                                dt
            1                                 0
            0                                 1];
 
    x = [robot.x; robot.y; robot.theta; robot.vx; robot.w];
    x_star = F * x + B * u';
    robot.x = x_star(1); robot.y = x_star(2); robot.theta = x_star(3);
    robot.vx = x_star(4); robot.w = x_star(5);
end