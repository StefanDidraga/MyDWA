#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include "Functions/Vec2CSV.hpp"

using namespace std;

struct Point {
    double x, y;
};



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

int main()
{

    //x(theta) = a*e^(b*theeta) * cos(theta)  
    //y(theta) = a*e^(b*theeta) * sin(theta)

    Point robot = {2.0, 2.0};
    vector<Point> obstacles = {};
    vector<double> obsx = {};
    vector<double> obsy = {};

    for (double theta = 0.0; theta <= 4 * M_PI; theta += 0.05) {
        double a = 0.5;
        double b = 0.2;
        Point obs;
        obs.x = a * exp(b * theta) * cos(theta);
        obsx.push_back(double(obs.x));
        obs.y = a * exp(b * theta) * sin(theta);
        obsy.push_back(double(obs.y));
        obstacles.push_back(obs);
    }

    vector<vector <double>> data2file = {obsx, obsy}; 

    double min_dist = min_dist_to_obs(robot, obstacles);
    std::cout << "Minimum distance to obstacles: " << min_dist << std::endl;

    writeToCSV("Distance2obs.csv", data2file);
}