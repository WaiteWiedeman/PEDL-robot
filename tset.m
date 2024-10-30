clc; clear; close all;

%% params
ctrlParams = params_control();
sysParams = params_system();

%% random points
t=0:0.01:5;
x = ctrlParams.a*cos((2*pi/5)*t);
y = ctrlParams.b*sin((2*pi/5)*t);

endx = 3 + 2*cos((2*pi/5)*t);
endy = 2*sin((2*pi/5)*t);

for i = 1:100
    theta = 2*pi*rand;
    rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad*cos(theta);
    ctrlParams.refy = ctrlParams.b*rad*sin(theta);
    Xd(i) = ctrlParams.refx;
    Yd(i) = ctrlParams.refy;
end

plot(x,y,'k-',endx,endy,'b-',Xd,Yd,'*')

%% IK check
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = -4; %ctrlParams.a*rad*cos(theta);
ctrlParams.refy = ctrlParams.b*rad*sin(theta);
[Xd, Yd, Xdd, Ydd, Xc, Xcd] = referenceTrajectory(0,ctrlParams);
[Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xc,Xcd);
[x1,y1,x2,y2,xend1,yend1,xend2,yend2] = ForwardKinematics([Xc Th1 Th2],sysParams);
