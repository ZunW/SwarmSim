%% a test script for simulator class
clear all;
close all;
% generate map for the simulation
size = 15;
resolution = 10;
numObstacles = 0;
space = 5;
%p = zeros(size*resolution);
map_gen = MapGenerate(size,size,space,resolution);
[p,map_gen] = map_gen.addBounds(2);
for i = 1:numObstacles
    %p = add_random_circle(p);
    [p,map_gen] = map_gen.addRandomObstacle(1.0,0.5);
end
map = binaryOccupancyMap(p,resolution);

%% specify some parameters
numRobots = 7;
numSensors = 25;
sensorRange = 2;
showTraj = false;
% initial_poses = [6; 6; 0];
initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
% initial_poses = [4.1688    3.5149    0.8034    3.6167    2.7753    1.8485;    1.6434    2.0218    0.7158    4.2360    2.3776    1.1487;0         0         0         0         0         0];
% initial_poses = [0.5901    1.1459    0.8769    3.2838    0.6344;    2.2010    1.2151    2.8941    3.2996    0.7752;         0         0         0         0         0];
% initial_poses = [4 1 1 2 2; 1.5 1 2 1 2; 0         0         0         0         0];
% initial_poses = [1.2368    1.0365    0.7858    2.2669    1.2866;2.8888    1.3504    1.4699    0.5531    0.8735;0         0         0         0         0];
% initial_poses = [4.2248    0.7536    4.4376    2.5535    1.0357    1.7052;    3.4146    3.9418    3.9358    1.2104    0.6236    1.6821;0         0         0         0         0         0];
disp(initial_poses);

robotInfos = cell(1,numRobots);
for i = 1:numRobots
    t = "DiffDrive"; % differential drive dynamics
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    robotInfos{i} = robotInfo;
end
swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
%% virtual structure simulation
sim = BehaviorBasedSimulation(map,swarmInfo);
numSim = 400;
numVip = 2;
distance1All = zeros([1 numSim]);
distance2All = zeros([1 numSim]);
for i = 1:numSim
    
    %calculate the "centroid"
    poses = sim.world.poses;
    positions = poses(1:2,:);
    total_position = [0; 0];
    for j = 3:numRobots
        position = positions(:,j);
        total_position = total_position + position;
    end
    centroid = total_position/(numRobots-2); %in world coord
    dist1 = norm(positions(:,1)-centroid);
    dist2 = norm(positions(:,2)-centroid);
    distance1All(i) = dist1;
    distance2All(i) = dist2;
    
    if i == numSim/2
        dummy = 1;
    end
    

    sim = sim.step();        
    axis([0 15 0 15])    
    
    if i == 1
        dummy = 1;
    end
    
    pause(0.02);
end


%% display the measure
figure('Name','VIP distance to centroid');
x = linspace(1,numSim,numSim);
tiledlayout(2,1)

nexttile
plot(x, distance1All);
title("VIP 1 distance to centroid");
xlabel('time step') 
ylabel('VIP 1 distance to centroid [meters]') 
axis([0 numSim 0 2])

nexttile
plot(x, distance2All);
title("VIP 2 distance to centroid");
xlabel('time step') 
ylabel('VIP 2 distance to centroid [meters]') 
axis([0 numSim 0 2])

% function p = add_random_circle(p)
%     world_size = size(p,1);
%     circle_size = floor(world_size/10);
%     x = rand*(2*circle_size) - 1*circle_size + world_size/2;
%     y = rand*(2*circle_size) - 1*circle_size + world_size/2;
%     xy = [x y];
%     for i = 1:world_size
%         for j = 1:world_size
%             dist = sqrt((i-x)^2 + (y-j)^2);
%             if (dist < circle_size)
%                 p(i,j) = 1;
%             end
%         end
%     end
% end