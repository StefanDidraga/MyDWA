function [v_x, w] = my_dwa_nopath(robot, goal, obs)
%my atempt at writing a dynamic window approach planner
%global planer
%obs is a n x 2 matrix with all the coordinates of the obstacles
% robot.x;
% robot.y;
% robot.theta;
% robot.vx;
% robot.w;



    limits.v_x_max = 1; % [m/s]
    limits.v_y_max = 0.4; % [m/s]
    limits.w_max = 0.5; % [rad/s]
    limits.a_x_max = 3.4; % [m/s^2]
    limits.a_y_max = 3.4; % [m/s^2]
    limits.wdot = 0.5; % [rad/s^2]
    limits.v_x_res = 0.01; % velocity resolution x [m/s]
    limits.w_res = 0.02; % angular velocity resolution [rad/s]
    
    dt = 0.1; % [s]

    % evalution parameters    [heading, distance, velocity, predict_time, R]
    % R - if the distance is to an object is larger then R it just takes R
    parameters = [0.065, 1 ,0.75, 3.0, 2];

    vr = calc_dynamic_win(robot, limits, dt);
    [eval_win] = evaluation(robot, vr, goal, obs, limits, parameters, dt);
    
    value = eval_win(:, 3);
    [~, index] = max(value);
    u = eval_win(index, 1:2);
    
    v_x = u(1,1);
    w = u(1,2);

end

function vr = calc_dynamic_win(robot, limits, dt)
%calculate dynamic window
    % hard margin
    vs=[0 , limits.v_x_max, ...
        -limits.w_max, limits.w_max];
    % predict margin
    vd = [robot.vx - limits.a_x_max * dt, robot.vx + limits.a_x_max * dt, ...
           robot.w - limits.wdot * dt, robot.w + limits.wdot * dt];
    % dynamic window
    v_tmp = [vs; vd];
    vr = [max(v_tmp(:, 1)) min(v_tmp(:, 2)) max(v_tmp(:, 3)) min(v_tmp(:, 4))];
end


function [eval_win] = evaluation(robot, vr, goal, obstacle, limits, parameters, dt)
    eval_win = [];
    for v = vr(1):limits.v_x_res:vr(2)
        for w=vr(3):limits.w_res:vr(4)
            % trajectory prediction, consistent of poses
            [robot_star] = generate_traj(robot, v, w, parameters(4), dt);

            
            % heading evaluation (still uses final pose)
            theta = angle([robot_star.x, robot_star.y], goal(1:2));
            heading = pi - abs(robot_star.theta - theta);
            
            dist_vector = dist(obstacle, [robot_star.x; robot_star.y]);
            distance = min(dist_vector);
                if distance > parameters(5)
                    distance = parameters(5);
                end

            % velocity evaluation
            velocity = abs(v);
            
            % braking evaluation
            dist_stop = v * v / (2 * limits.a_x_max);

            % collision check
            if distance > dist_stop && distance >= 0.1
                eval_win = [eval_win; [v w heading distance velocity]];
            end
        end
    end
    
    if isempty(eval_win)
        error('DWA:NoSafePath', 'No safe velocity found. The robot is likely trapped by obstacles.');
    end

    % normalization
    if sum(eval_win(:, 3)) ~= 0
        eval_win(:, 3) = eval_win(:, 3) / sum(eval_win(:, 3));
    end
    if sum(eval_win(:, 4)) ~= 0
        eval_win(:, 4) = eval_win(:, 4) / sum(eval_win(:, 4));
    end
    if sum(eval_win(:, 5)) ~= 0
        eval_win(:, 5) = eval_win(:, 5) / sum(eval_win(:, 5));
    end
    eval_win = [eval_win(:, 1:2), eval_win(:, 3:5) * parameters(1:3)'];
end


function [robot] = generate_traj(robot, v, w, t, dt)
    time = 0;
    u = [v, w];    
    while time <= t   
        time = time + dt;
        robot = f(robot, u, dt);
    end
end

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

function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
end