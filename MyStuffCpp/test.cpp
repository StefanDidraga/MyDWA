#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include "Tests/Functions/Vec2CSV.hpp"

using namespace std;

// --- Data Structures ---
struct RobotState {
    double x, y, theta, vx, w;
};

struct Point {
    double x, y;
};

struct Limits {
    double v_x_max = 1.0;     // [m/s]
    double v_y_max = 0.4;     // [m/s]
    double w_max = 0.5;       // [rad/s]
    double a_x_max = 3.4;     // [m/s^2]
    double a_y_max = 3.4;     // [m/s^2]
    double wdot = 0.5;        // [rad/s^2]
    double v_x_res = 0.01;    // [m/s]
    double w_res = 0.02;      // [rad/s]
};

struct Parameters {
    double heading_weight = 0.065;
    double dist_weight = 1.0;
    double vel_weight = 0.75;
    double predict_time = 3.0; // [s]
    double max_dist = 2.0;     // [m]
};

struct EvalRecord {
    double v, w, heading, distance, velocity, score;
};

// --- Helper Functions ---

// Calculate angle between two points
double calc_angle(const Point& n1, const Point& n2) {
    return atan2(n2.y - n1.y, n2.x - n1.x);
}

// Calculate minimum distance from a point to a list of obstacles
double min_dist_to_obs(const Point& p, const vector<Point>& obs) {
    double min_d = numeric_limits<double>::max();
    for (const auto& o : obs) {
        double d = hypot(p.x - o.x, p.y - o.y); // hypotenuse: sqrt(dx^2 + dy^2)
        if (d < min_d) {
            min_d = d;
        }
    }
    return min_d;
}

// Kinematic model (direct scalar math instead of matrices for speed)
RobotState f(RobotState state, double v, double w, double dt) {
    state.x += v * cos(state.theta) * dt;
    state.y += v * sin(state.theta) * dt;
    state.theta += w * dt;
    state.vx = v;
    state.w = w;
    return state;
}

// Generate trajectory
RobotState generate_traj(RobotState state, double v, double w, double t, double dt) {
    double time = 0.0;
    while (time <= t + 1e-5) {
        time += dt;
        state = f(state, v, w, dt);
    }
    return state; // Returns the final pose
}

// Calculate dynamic window
vector<double> calc_dynamic_win(const RobotState& robot, const Limits& limits, double dt) {
    // [v_min, v_max, w_min, w_max]
    double v_min = max(0.0, robot.vx - limits.a_x_max * dt);
    double v_max = min(limits.v_x_max, robot.vx + limits.a_x_max * dt);
    double w_min = max(-limits.w_max, robot.w - limits.wdot * dt);
    double w_max = min(limits.w_max, robot.w + limits.wdot * dt);
    
    return {v_min, v_max, w_min, w_max};
}

// Evaluate window and find the best velocities
vector<double> evaluation(const RobotState& robot, const vector<double>& vr, 
                          const Point& goal, const vector<Point>& obstacles, 
                          const Limits& limits, const Parameters& params, double dt) {
    
    vector<EvalRecord> eval_win;
    double sum_heading = 0.0;
    double sum_distance = 0.0;
    double sum_velocity = 0.0;

    for (double v = vr[0]; v <= vr[1] + 1e-5; v += limits.v_x_res) {
        for (double w = vr[2]; w <= vr[3] + 1e-5; w += limits.w_res) {
            
            // Trajectory prediction
            RobotState robot_star = generate_traj(robot, v, w, params.predict_time, dt);
            
            // Heading evaluation
            double theta = calc_angle({robot_star.x, robot_star.y}, goal);
            double heading = M_PI - abs(robot_star.theta - theta);
            
            // Distance evaluation
            double distance = min_dist_to_obs({robot_star.x, robot_star.y}, obstacles);
            if (distance > params.max_dist) {
                distance = params.max_dist;
            }
            
            // Velocity evaluation
            double velocity = abs(v);
            
            // Braking evaluation (collision check)
            double dist_stop = (v * v) / (2 * limits.a_x_max);
            
            if (distance > dist_stop && distance >= 0.1) {
                eval_win.push_back({v, w, heading, distance, velocity, 0.0});
                sum_heading += heading;
                sum_distance += distance;
                sum_velocity += velocity;
            }
        }
    }

    if (eval_win.empty()) {
        throw runtime_error("DWA:NoSafePath - No safe velocity found. The robot is likely trapped.");
    }

    // Normalization and scoring
    double best_score = -1.0;
    double best_v = 0.0;
    double best_w = 0.0;

    for (auto& record : eval_win) {
        if (sum_heading != 0) record.heading /= sum_heading;
        if (sum_distance != 0) record.distance /= sum_distance;
        if (sum_velocity != 0) record.velocity /= sum_velocity;

        record.score = (record.heading * params.heading_weight) + 
                       (record.distance * params.dist_weight) + 
                       (record.velocity * params.vel_weight);

        if (record.score > best_score) {
            best_score = record.score;
            best_v = record.v;
            best_w = record.w;
        }
    }

    return {best_v, best_w};
}

// --- Main DWA Function ---
vector<double> my_dwa_nopath(const RobotState& robot, const Point& goal, const vector<Point>& obs) {
    Limits limits;
    Parameters params;
    double dt = 0.1; // [s]

    vector<double> vr = calc_dynamic_win(robot, limits, dt);
    vector<double> best_u = evaluation(robot, vr, goal, obs, limits, params, dt);

    return best_u; // Returns [v_x, w]
}

// --- Example Usage ---
int main() {
    RobotState my_robot = {0.0, 0.0, M_PI/4, 0.0, 0.0};
    Point my_goal = {10.0, 10.0};
    double dt = 0.1;
    
    vector<Point> obstacles = {
        {5.0, 5.0},
        {4.0, 6.0},
        {5.0, 6.0}
    };
    double sim_time=0;
    double maxtime = 20.0;
    double max_dist = 0.2;
    int k = 1;


    vector<double> roboX = {};
    vector<double> roboY = {};
    vector<double> robotheta = {};
    vector<vector <double>> data2file = {};

    while (sim_time < maxtime)
    {
        vector<double> vel = my_dwa_nopath(my_robot, my_goal, obstacles);
        my_robot = f(my_robot, vel[0], vel[1], dt);

        // random tests start
        Point robot_pos;
        robot_pos.x = my_robot.x;
        roboX.push_back(robot_pos.x);
        robot_pos.y = my_robot.y;
        roboY.push_back(robot_pos.y);
        robotheta.push_back(my_robot.theta);
        cout << k << " : " << my_robot.x << "   " << my_robot.y << "  " << my_robot.theta<< "\n";
        double distance2obs = min_dist_to_obs(robot_pos, (vector<Point> {my_goal}));
        // cout << "Distance to goal "<< distance2obs << "\n";
        k++;
        if(distance2obs < max_dist)
        {
            cout << "Goal reached in "<< sim_time<< "\n";
            break;
        }
        // random tests end

        sim_time = sim_time + dt;
    }
    cout << "Sim time "<< sim_time<< "\n";
   
    data2file = {roboX, roboY, robotheta};
    writeToCSV("DWA_traj_Cpp.csv", data2file);
    
    return 0;
}