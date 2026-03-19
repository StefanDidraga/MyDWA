clear all;
clc;

addpath("MyStuffCpp\Tests\");

robot = [0,0];
obs=[[5,5];[4,6];[5,6];[4,8]];

dist_vector = dist(robot,obs');

% for i=1:(length(obs))
%     fprintf("%d : distance = %5.3f \n",i,dist_vector(i));
% end

min_dist = min(dist_vector);

fprintf("min distance: %5.3f", min_dist);

data = readmatrix("Distance2obs.csv");
x = data(1,:);
y = data(2,:);

figure
plot(x,y);
legend("Logarithmic Spiral")