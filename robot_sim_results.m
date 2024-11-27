%% clear workspace
close all; clear; clc;

%% Two-link Planar Robot on Cart Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
ctrlParams.method = "origin";
ctrlParams.solver = "GA"; % "stiff" or "nonstiff"
tSpan = [0,30]; %0:0.01:5;
% % ctrlParams.PID0 = [10 0 0]; 
% % ctrlParams.PID1 = [10 0 0];
% % ctrlParams.PID2 = [10 0 0];

%% run simulation and plot states, forces, and states against reference
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = ctrlParams.a*rad*cos(theta);
ctrlParams.refy = ctrlParams.b*rad*sin(theta);
% x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
% x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
x0 = [0; 0; 0; 0; 0; 0]; % th0, th0d, th1, th1d, th2, th2d

y = robot_simulation(tSpan, x0, sysParams, ctrlParams);

% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:10),"position",y(:,[19 17 18]));
plot_states(y(:,1),y(:,2:10),"velocity",y(:,15:18));
plot_states(y(:,1),y(:,2:10),"acceleration",y(:,15:18));
plot_forces(y(:,1),y(:,11),y(:,12),y(:,13),y(:,14));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:4),sysParams);
plot_endeffector([xend yend],y(:,15:16)) %y(:,15:16)

info1 = lsiminfo(y(:,2),y(:,1));
info2 = lsiminfo(y(:,3),y(:,1));
info3 = lsiminfo(y(:,4),y(:,1));

disp(xend(end))
disp(yend(end))
disp(y(end,2)-y(end,19))
disp(y(end,3)-y(end,17))
disp(y(end,4)-y(end,18))

%% run simscape model and plot states, forces, and states against reference for simscape model
y_simscape = run_simscape();

plot_states(y_simscape(:,1),y_simscape(:,2:10));
plot_forces(y_simscape(:,1),y_simscape(:,11),y_simscape(:,12),y_simscape(:,13),y_simscape(:,14));
plot_reference(y_simscape(:,1),y_simscape(:,2:4),y_simscape(:,15:18))

% solve forward kinematics and plot end effector position for simscape model
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(y_simscape(:,2:4),sysParams);
plot_endeffector([xend yend],y_simscape(:,15:16)) %y(:,15:16)

%% Tune PID w/ GA
N_monte_carlo = 100;
tSpan = [0,60]; 
POlim = 0.5;
Ts_lim = tSpan(2)-10;
Rise_thresh = 10;
myObj = @(gene) fitnessfun(gene, N_monte_carlo,tSpan,POlim,Ts_lim,Rise_thresh,ctrlParams,sysParams);
% GA parameters
Pc = 0.80;
fitfun = @(x) myObj(x);
PopSize = 200;
MaxGens = 100;
nvars   = 9;
A       = [];
b       = [];
Aeq     = [];               
beq     = [];
lb      = 10*ones(1,9);
ub      = 10000*ones(1,9);
nonlcon = [];
options = optimoptions('ga', 'PopulationSize', PopSize, 'MaxGenerations',...
    MaxGens,'PlotFcn',{@gaplotbestf,@gaplotscores});
options.CrossoverFraction = Pc;
% GA to search for near-optimal solution
start = datetime
[BestChrom, fval, exitflag, output] = ga(fitfun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);
stop = datetime
disp(stop-start)

%% simulate PID controller tuned by GA
ctrlParams.PID0 = BestChrom(1:3); 
ctrlParams.PID1 = BestChrom(4:6);
ctrlParams.PID2 = BestChrom(7:9);

%% coulomb friction
v = linspace(-5,5,10000);
fc = coulomb_friction(v, sysParams, ctrlParams.friction);
idx = find(v >= 0, 1, 'first');
slopeangle = atan((fc(idx))/(v(idx)))*180/pi;
figure('Position',[500,100,800,800]);
plot(v,fc,'b-','LineWidth',2);
xlabel("velocity (m/s)");
ylabel('Coulomb Friction Force (N)');
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
disp(slopeangle)

%% random points
t=0:0.01:5;
x = ctrlParams.a*cos((2*pi/5)*t);
y = ctrlParams.b*sin((2*pi/5)*t);

numCase = 100;
theta = linspace(0,2*pi,numCase);
rad = linspace(0,1,numCase);

for i = 1:numCase
    % theta = 2*pi*rand;
    % rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad(i)*cos(theta(i));
    ctrlParams.refy = ctrlParams.b*rad(i)*sin(theta(i));
    Xd(i) = ctrlParams.refx;
    Yd(i) = ctrlParams.refy;
end

figure('Position',[500,100,800,800]);
plot(x,y,'k-',Xd,Yd,'b*','LineWidth',1)
axis padded
ylabel("Y");
xlabel("X");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
legend("Objective Bounds", "Random Objective Point")

%%
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,2),'b-o','LineWidth',2);
ylabel("$\theta_0$","Interpreter","latex");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', "Arial")
xlabel("Time (s)");

%% functions
function plot_forces(t,u,t1,t2,fc)
    figure('Position',[500,100,800,800]);
    plot(t,u,'k-',t,t1,'b-',t,t2,'g-',t,fc,'m-','LineWidth',2);
    legend("u","$\tau_1$","$\tau_2$","$f_c$","Interpreter","latex");
end

function plot_states(t,x,flag,refs)
labels= ["$\theta_0$","$\theta_1$","$\theta_2$","$\dot{\theta}_0$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
    "$\ddot{\theta}_0$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
switch flag
    case "position"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 1:numState-6
            nexttile
            plot(t,x(:,i),'b-',t,refs(:,i),'k:','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-6
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Reference","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 4:numState-3
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-3
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 7:numState
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end

function plot_endeffector(x,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'*','LineWidth',2);
    end
    axis padded
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end

function [fit]  = fitnessfun(gene, N_monte_carlo,tSpan,POlim,Ts_lim,Rise_thresh,ctrlParams,sysParams)

ctrlParams.PID0 = gene(1:3);
ctrlParams.PID1 = gene(4:6);
ctrlParams.PID2 = gene(7:9);

ts1_above_thresh = zeros(1,N_monte_carlo);
ts2_above_thresh = zeros(1,N_monte_carlo);
ts3_above_thresh = zeros(1,N_monte_carlo);
x1_above_thresh = zeros(1,N_monte_carlo);
x2_above_thresh = zeros(1,N_monte_carlo);
x3_above_thresh = zeros(1,N_monte_carlo);
rs1_above_thresh = zeros(1,N_monte_carlo);
rs2_above_thresh = zeros(1,N_monte_carlo);
rs3_above_thresh = zeros(1,N_monte_carlo);
e1 = zeros(1,N_monte_carlo);
e2 = zeros(1,N_monte_carlo);
e3 = zeros(1,N_monte_carlo);
exend = zeros(1,N_monte_carlo);
eyend = zeros(1,N_monte_carlo);

for sim_n = 1:N_monte_carlo
    theta = 2*pi*rand;
    rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad*cos(theta);
    ctrlParams.refy = ctrlParams.b*rad*sin(theta);
    x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
    x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
    y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
    [~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:4),sysParams);

    if y(end,1) < tSpan(2)
        break
    end

    e1(sim_n) = abs(y(end,2) - y(end,19));
    e2(sim_n) = abs(y(end,3) - y(end,17));
    e3(sim_n) = abs(y(end,4) - y(end,18));
    exend(sim_n) = abs(ctrlParams.refx - xend(end));
    eyend(sim_n) = abs(ctrlParams.refy - yend(end));

    info1 = lsiminfo(y(:,2),y(:,1));
    info2 = lsiminfo(y(:,3),y(:,1));
    info3 = lsiminfo(y(:,4),y(:,1));

    Ts1 = info1.SettlingTime;
    Ts2 = info2.SettlingTime;
    Ts3 = info3.SettlingTime;

    if x0(1) >= y(end,19)
        max1 = info1.Min;
        Rt1 = info1.MinTime;
    else 
        max1 = info1.Max;
        Rt1 = info1.MaxTime;
    end

    if x0(3) >= y(end,17)
        max2 = info2.Min;
        Rt2 = info2.MinTime;
    else 
        max2 = info2.Max;
        Rt2 = info2.MaxTime;
    end

    if x0(5) >= y(end,18)
        max3 = info3.Min;
        Rt3 = info3.MinTime;
    else
        max3 = info3.Max;
        Rt3 = info3.MaxTime;
    end

    if Ts1 > Ts_lim  
        ts1_above_thresh(sim_n) = 1;
    elseif Ts2 > Ts_lim
        ts2_above_thresh(sim_n) = 1;
    elseif Ts3 > Ts_lim
        ts3_above_thresh(sim_n) = 1;
    end

    if abs(max1 - y(end,19))/abs(x0(1) - y(end,19)) > POlim 
        x1_above_thresh(sim_n) = 1;
    elseif abs(max2 - y(end,17))/abs(x0(3) - y(end,17)) > POlim
        x2_above_thresh(sim_n) = 1;
    elseif abs(max3 - y(end,18))/abs(x0(5) - y(end,18)) > POlim
        x3_above_thresh(sim_n) = 1;
    end

    if Rt1 > Rise_thresh 
        rs1_above_thresh(sim_n) = 1;
    elseif Rt2 > Rise_thresh
        rs2_above_thresh(sim_n) = 1;
    elseif Rt3 > Rise_thresh
        rs3_above_thresh(sim_n) = 1;
    end
end

% fitness is weighted squared probability
if sim_n < N_monte_carlo
    fit = 1e5;
else
    fit = (1e2)*((sum(x1_above_thresh)/N_monte_carlo)^2 + (sum(x2_above_thresh)/N_monte_carlo)^2 + (sum(x3_above_thresh)/N_monte_carlo)^2 ...
        + (sum(ts1_above_thresh)/N_monte_carlo)^2 + (sum(ts2_above_thresh)/N_monte_carlo)^2 + (sum(ts3_above_thresh)/N_monte_carlo)^2 ...
        + (sum(rs1_above_thresh)/N_monte_carlo)^2 + (sum(rs2_above_thresh)/N_monte_carlo)^2 + (sum(rs3_above_thresh)/N_monte_carlo)^2 ...
        + 10*mean(e1) + 10*mean(e2) + 10*mean(e3) + 100*mean(exend) + 100*mean(eyend)); % 
end
end
