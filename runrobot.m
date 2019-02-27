function [t]=runrobot()
close all; clear all;
fismat = readfis('FCproject.fis'); % This is where you will load your fuzzy rules

% Below are a few examples of how to encode the goal
goal = [10 10]; goald = .3; % Use this line for 6a and 6b

goal = [20 20; 5 20]; goald = [1; 1]; % 6c

goal = [0 10; 0 0; 20 0]; goald = [4; 1; 5]; % 6d

goal = [5 5; 4 4; 6 6; 3 6; 6 3]; goald = [.5 .5 .5 .5 .5]; % 6e

n=size(goal,1);
robot.x = [0 0];
robot.ang = 0;
robot.v = [0 0];
robot.a = [0 0];

figure(1); axis equal
viscircles(goal,goald); hold on;
quiver(robot.x(1),robot.x(2),cosd(robot.ang),sind(robot.ang),'b');

dt=0.1;
t=0;
for g=1:n,
    scl = goald(g); % scales distance by size of goal
    disp(['Seeking goal ',num2str(g)]);
    d = sqrt(distance2(robot.x,goal(g,:)));
    gx = goal(g,:)-robot.x;
    vec = gx*[cosd(robot.ang) -sind(robot.ang); sind(robot.ang) cosd(robot.ang)];
    ang = atan2d(vec(2),vec(1));
    while(d > goald(g)),
        robot = updateR(robot,dt);
        robot.a = evalfis([d/scl ang robot.v(1) robot.v(2)],fismat);
        
        if(~mod(t,10))
            clf;
            viscircles(goal,goald); axis equal; hold on;
            plot(robot.x(1),robot.x(2),'bo');
            quiver(robot.x(1),robot.x(2),2*cosd(robot.ang),2*sind(robot.ang),'b');
            drawnow;
        end;
        
        d = sqrt(distance2(robot.x,goal(g,:)));
        gx = goal(g,:)-robot.x;
        vec = gx*[cosd(robot.ang) -sind(robot.ang); sind(robot.ang) cosd(robot.ang)];
        ang = atan2d(vec(2),vec(1));
        t=t+1;
    end;
    
    
end;


function [robot] = updateR(robot,dt)
% updates robot position after dt seconds
% robot.a(1) - radial acceleration
% robot.a(2) - angular acceleration
% robot.v(1) - radial velocity
% robot.v(2) - angular velocity
% robot.x - (horizontal, vertical) position
% robot.ang - angle in degrees

robot.v = robot.v + robot.a*dt;
if(robot.v(1)>1)
    robot.v(1)=1;
elseif(robot.v(1)<0)
    robot.v(1)=0;
end;
if(abs(robot.v(2))>60)
    robot.v(2)=sign(robot.v(2))*60;
end;
robot.x = robot.x+[cosd(robot.ang)*robot.v(1)*dt sind(robot.ang)*robot.v(1)*dt];
robot.ang = robot.ang + robot.v(2)*dt;
robot.ang = mod(robot.ang-180,360)-180;

return;