% clc
% clear all
% path = [2.00    1.00;
%         1.25    1.75;
%         5.25    8.25;
%         7.25    8.75;
%         11.75   10.75;
%         12.00   10.00];
%当前与目标位置
% robotInitialLocation = path(1,:);
% robotGoal = path(end,:);
%初始方向
% initialOrientation = 0;
%%当前姿势
% robotCurrentPose = [robotInitialLocation initialOrientation]';
%创建机器人模型
% robot = differentialDriveKinematics("TrackWidth", 3, "VehicleInputs", "VehicleSpeedHeadingRate");
%可视化轨迹
% % figure
% % plot(path(:,1), path(:,2),'k--d')
% % xlim([0 13])
% % ylim([0 13])

%路径跟随器
% controller = controllerPurePursuit;
% controller.Waypoints = path;%路径
% controller.DesiredLinearVelocity = 0.6;%线速度
% controller.MaxAngularVelocity = 2;%角速度
% controller.LookaheadDistance = 0.3;
% % %目标半径
% % goalRadius = 0.1;
% % distanceToGoal = norm(robotInitialLocation - robotGoal);
% % %
% % % %初始化循环
% sampleTime = 0.1;
% vizRate = rateControl(1/sampleTime);
% % 
% % figure%可视化
% frameSize = robot.TrackWidth/0.8;
% % 
% % while( distanceToGoal > goalRadius )
% %     
% %     % Compute the controller outputs, i.e., the inputs to the robot
% %     [v, omega] = controller(robotCurrentPose);    
% %     % Get the robot's velocity using controller inputs
% %     vel = derivative(robot, robotCurrentPose, [v omega]);%速度，
% %     %更新姿态
% %     robotCurrentPose = robotCurrentPose + vel*sampleTime;  
% %     % 重新计算距离
% %     distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));  
% %     hold off  
% %     % Plot path each instance so that it stays persistent while robot mesh
% %     % 移动
% %     plot(path(:,1), path(:,2),"k--d")
% %     hold all
% %     
% %      % Plot the path of the robot as a set of transforms
% %      plotTrVec = [robotCurrentPose(1:2); 0];
% %      plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
% %      plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
% %      light;
% %      xlim([0 13])
% %      ylim([0 13])
% %      waitfor(vizRate);
% % end

robot = differentialDriveKinematics("TrackWidth", 1.5, "VehicleInputs", "VehicleSpeedHeadingRate");
% I = imread('way.jpg');
% J = imresize(I, 0.4);
% thresh = graythresh(J);
% I2 = im2bw(J,thresh); %对图像二值化
% I3=imcomplement(I2)
% figure(1);
% subplot(1,2,1);  
% imshow(J);    %显示二值化之前的图片
% title('原图'); 
% subplot(1,2,2);
% imshow(I3);    %显示二值化之后的图片
% title('二值化');





%加载地图
load exampleMaps
map = binaryOccupancyMap(simpleMap);
figure(1)
show(map)
% PRM路径规划算法
mapInflated = copy(map);
inflate(mapInflated, robot.TrackWidth/2);
prm = robotics.PRM(mapInflated);
% way1
% % % prm.NumNodes = 800;
% % % prm.ConnectionDistance = 20;
 prm.NumNodes = 500;
 prm.ConnectionDistance = 10;
% 
% prm.NumNodes = 3000;
% prm.ConnectionDistance = 30;
% % way1
% % startLocation = [10 250];
% % endLocation = [390 5];
startLocation = [5 6];
endLocation = [22 3.5];


path = findpath(prm, startLocation, endLocation);
show(prm);
% 
controller = controllerPurePursuit;
controller.Waypoints = path;%路径
controller.DesiredLinearVelocity = 3;%线速度
controller.MaxAngularVelocity = 2;%角速度
controller.LookaheadDistance = 0.3;

robotInitialLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

distanceToGoal = norm(robotInitialLocation - robotGoal);

goalRadius = 0.1;

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
frameSize = robot.TrackWidth/0.8;

reset(vizRate);

% Initialize the figure
figure(2)

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % 速度计算
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    %更新位置
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % 目标距离
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(map);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
%     xlim([0 27])
%     ylim([0 26])
    
    waitfor(vizRate);
end