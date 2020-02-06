
clear; 
clc; 
clf; 
close all; 
% ------------------------------ Setup ------------------------------- %
global programOn;    programOn  = 1;
global viewStyle;    viewStyle  = 0;                        % Mouse

global mouseClick;   mouseClick = 0;                        
global mouseDown;    mouseDown  = 0;
global mouseUp;      mouseUp    = 0;
robot = robotMake(  1,  1, .5);

%  robotPaint(robot);

% ----------------------------- Interact Demo -------------------------- %

% -------------------------- Auto Demo Function ------------------------ %

numPts=160;  % Define Pts 
spiralPts = shapePts([  .5;   -1;  -.48], .25, numPts, 'spiral');   
flowerPts = shapePts([  .5;  -.3;  -.48], .25, numPts, 'flower');
squarePts = shapePts([ 1.2;  -.3;  -.48], .25, numPts, 'square');
circlePts = shapePts([ 1.2;   -1;  -.48], .25, numPts, 'circle');    
sinePts   = [                linspace(.4, 1.3, numPts); 
  .2*ones(1,numPts) + .1*sin(linspace(0,8*pi, numPts));
                                  -.48*ones(1,numPts)];
                                                            % Draw Pts
robotMove(robot, [1; -.5; -.48], spiralPts,numPts);            
robotMove(robot, spiralPts); robotMove(robot, spiralPts, flowerPts, numPts);
robotMove(robot, flowerPts); robotMove(robot, flowerPts, squarePts, numPts);
robotMove(robot, squarePts); robotMove(robot, squarePts, circlePts, numPts); 
robotMove(robot, circlePts); robotMove(robot, circlePts,   sinePts, numPts); 
robotMove(robot, sinePts);   robotMove(robot, sinePts,[1;-.5;-.48], numPts);

function robotPaint(robot)
global mouseClick mouseUp mouseDown
clearpoints(robot.brushLine);
set(robot.fig, 'Visible', 'off')
set(robot.altFig, 'Visible', 'on')
ptsNum = 0;
pts = [0;0;0]
while mouseUp == 0
    [x,y] = uiFunction(robot);
    if mouseDown == 1
        ptsNum = ptsNum + 1;
        pts(:,ptsNum) = [x;y;-.48];
        addpoints(robot.brushLine, x,y);
    end
    pause(.01);
end
mouseUp = 0;
set(robot.altFig, 'Visible', 'off')
set(robot.fig, 'Visible', 'on')

pts = pts + repmat([.3; -1.1;0], 1,numel(pts)/3);

robotMove(robot, [1; -.5; -.48], pts,60);  
robotMove(robot, pts);
robotMove(robot, pts, [1; -.5; -.48],60);
end
% ------------------------- Generate Robot ----------------------------- %
function robot = robotMake(L1, L2, h)                                                            
syms R r x y z ang0 ang1 ang2                               % Solve                              
[R,    Ang0] = ...
    solve([x,y] == [ r*cos(ang0),  r*sin(ang0)], [r,ang0]);
[Ang1, Ang2] = ...
    solve([r,z] == [L1*cos(ang1) + L2*cos(ang2), ...
                    L1*sin(ang1) + L2*sin(ang2)],[ang1,ang2]);                                                                                                
robot.pos2angs = str2func(...                               % Make Fcns 
    strcat('@(x,y,z) [', string(Ang0(1)), ';', ...
                  strrep(string(Ang1(1)), 'r', string(R(1))), ';', ...
                  strrep(string(Ang2(1)), 'r', string(R(1))), ']'));
robot.angs2nodes = @(ang0,ang1,ang2) ...
  [0, 0, cos(ang0)*L1*cos(ang1), cos(ang0)*(L1*cos(ang1) + L2*cos(ang2));
   0, 0, sin(ang0)*L1*cos(ang1), sin(ang0)*(L1*cos(ang1) + L2*cos(ang2));  
   0, h,       h + L1*sin(ang1),        h + L1*sin(ang1) + L2*sin(ang2)];
    
set(0,'units','normalized');
robot.fig = figure( 'color',   [ .3, .35, .3], ...          
                    'units', 'normal', 'MenuBar',   'None');
robot.ax = axes('clipping',   'off',     'Projection', 'perspective', ...    
         'DataAspectRatio', [1 1 1],       'Position',   [.1 .1 .8 .8], ...
      'PlotBoxAspectRatio', [1 1 1], 'CameraPosition',     [ 2 -3  2], ...  
         'CameraViewAngle',      35,   'CameraTarget',     [ 1 -1 .5], ...
                 'Visible',   'off',         'parent',     robot.fig, ...
    'XLim', [-2 2], 'YLim',  [-1 4], 'ZLim',   [-2 2], 'units', 'normal', ...
    'PickableParts', 'none');
              
robot.base  = boxPatch([ .6; -.5; -.05],   1,   1, .05);    % Robot Base
robot.comp  = boxPatch([-.2; -.5;  .05],  .2,   1, .05);
robot.paper = boxPatch([ .8; -.5;  .01], .75, .92, .01);
set(robot.base,  'FaceColor',  [.1 .1 .1], 'FaceAlpha',1);
set(robot.comp,  'FaceColor',  [.5,.5,.7], 'FaceAlpha',1);
set(robot.paper, 'FaceColor',  [.9 .9 .9], 'FaceAlpha',1);

robot.angs    = robot.pos2angs(.5, -.1, -.48);               % Robot Arm
robot.nodes   = robot.angs2nodes(robot.angs(1), ...
                  robot.angs(2), robot.angs(3));
robot.arm     = line('linewidth',8, 'XData', robot.nodes(1,:), ...
                     'YData', robot.nodes(2,:), 'ZData', robot.nodes(3,:));                                                         
robot.drawing = animatedline('linestyle', 'none', ...       
    'marker', '.'); 

% Alternate Canvas
robot.altFig = figure('WindowButtonDownFcn',  @mouseDownCall,   'Color', [0 0 0], ...
                        'WindowButtonUpFcn',  @mouseUpCall, 'MenuBar',  'None', ...
                        'PointerShapeCData',  NaN(16,16), 'Visible', 'off');
       
robot.altAx = axes('Units',  'Normal',    'XLim', [0 1], ...          
      'Position', [0 0 1 1],    'YLim', [0 1], 'parent', robot.altFig, ...    
 'PickableParts',    'None', 'TickLen', [0 0], 'box', 'on')    

robot.brushLine = animatedline('color','black','linewidth', 8);
end

% ----------------------- Robot Draw Function ------------------------- %
function robotMove(robot,pts0, pts1, numPts)  
if nargin > 3                                            % define path pts
    udPts = ceil(numPts/4); 
    xyPts = ceil(numPts/2); 
    x0    = pts0(1,end); y0 = pts0(2,end); z0 = pts0(3,end);  z0Up = z0 + .2;  
    x1    = pts1(1,1);   y1 = pts1(2,1);   z1 = pts1(3,1);    z1Up = z1 + .2;
    pts =[repmat(x0,1,udPts), linspace(x0,x1,xyPts),      repmat(x1,1,udPts);
          repmat(y0,1,udPts), linspace(y0,y1,xyPts),      repmat(y1,1,udPts);
     linspace(z0,z0Up,udPts),  repmat(z0Up,1,xyPts), linspace(z1Up,z1,udPts)];
else
    pts = pts0;
end

for i = 1:numel(pts)/3   
    uiFunction(robot);
    angs  = robot.pos2angs(pts(1,i),pts(2,i),pts(3,i));   % Calc/Draw
    nodes = robot.angs2nodes(angs(1),angs(2),angs(3));
    set(robot.arm,  'XData', nodes(1,:), ...
                    'YData', nodes(2,:), 'ZData', nodes(3,:));
    if nargin < 3                                        
            addpoints(robot.drawing, nodes(1,end), ...
                      nodes(2,end),  nodes(3,end))
    end
    pause(.01);
end
pause(.05);
end
    
% ----------------------- Points Generator --------------------------- %
function pts = shapePts(center,radius,numPts,type)    
switch type
    case 'circle'
        pts = repmat(center,[1,numPts]) + ...
                     [radius*cosd(linspace(0,360,numPts));
                      radius*sind(linspace(0,360,numPts));
                                          zeros(1,numPts)];
    case 'square'
        c    = repmat(center,[1,numPts]);  
        r    = repmat(radius,[1,numPts/4]);
        span = linspace(-radius,radius,numPts/4);
        pts  = c +  [span,      r,  -span,     -r;
                       -r,   span,      r,  -span;
                                 zeros(1,numPts)];
    case 'flower'
        pts = repmat(center,[1,numPts]) + radius* ...
            [cosd(linspace(0,720,numPts)).*cosd(linspace(0,360,numPts));
             cosd(linspace(0,720,numPts)).*sind(linspace(0,360,numPts));
                                                       zeros(1,numPts)];
    case 'spiral'
        angles = linspace(  0,   5*pi,numPts);
        rVec   = linspace(.01, radius,numPts);
        pts    = repmat(center,[1,numPts]) + [rVec.*cos(angles);
                                              rVec.*sin(angles);
                                              zeros(1,numPts)];
end
end

% ------------------------- Box Maker Function ------------------------- %
function box = boxPatch(o,l,w,h)
vertices = repmat(o,1,8) + [-l -l -l -l  l  l  l  l  
                            -w -w  w  w -w -w  w  w
                            -h  h  h -h -h  h  h -h];
faces = [1 5 1 3 1 2; 
         2 6 2 4 4 3;
         3 7 6 8 8 7; 
         4 8 5 7 5 6]';  
box = patch('Vertices', vertices',     'Faces', faces,...
           'FaceColor', rand(1,3), 'FaceAlpha', .8);
end

% -------------------------- Mouse Functions ------------------------- %
function mouseDownCall(~,~)
global mouseClick; mouseClick = 1;
global mouseDown;  mouseDown  = 1;
end

function mouseUpCall(~,~)
global    mouseDown;  mouseDown = 0;
global      mouseUp;   mouseUp  = 1;
end

function [x,y] = uiFunction(robot)                                               
global mouseClick mouseDown mouseUp                     % Every Iteration
figPos = get(robot.fig,        'Position');                        
cPos   = get(        0, 'PointerLocation');        
x      = (cPos(1) - figPos(1))/(figPos(3));        
y      = (cPos(2) - figPos(2))/(figPos(4)); 
end