%% clear workspace
close all; clear; clc;

%% test variables
file = "best_dnn_models";
net = load(file).trainedNetwork;
sysParams = params_system();
ctrlParams = params_control();
trainParams = params_training();
trainParams.type = "dnn9"; % "dnn3","lstm3","pinn3","dnn6","lstm6","pinn6","dnn9", "lstm9","pinn9"
ctrlParams.method = "interval"; % random, interval, origin
numTime = 1000;
tSpan = 0:0.01:5; % [0,5] 0:0.01:5
predInterval = 5; 

%% simulation 
x0 = [0; 0; 0; 0; 0; 0]; % th0, th0d, th1, th1d, th2, th2d
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
t = y(:,1);
x = y(:,2:10);
[xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, trainParams.type,1);
% plot_compared_states(t,x,t,xp)
% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(x(:,1:3),sysParams);
[~,~,~,~,~,~,xpend,ypend] = ForwardKinematics(xp(:,1:3),sysParams);
% plot_endeffector([xend yend],[xpend ypend],0) %y(:,15:16)
% make image and video
tPred = [1,5];
% MakeImage(sysParams, t, x, xp, tPred)
MakeVideo(sysParams, t, x, xp, tPred)
disp(mean(rmseErr,'all'))

%% evaluate for four states
tSpan = [0,5];
predIntervel = 5;
numCase = 20;
numTime = 1000;
avgErr = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,1,0.5);
% avgErr = evaluate_model_with_4_states(net, sysParams, ctrlParams, trainParams, f1Max, tSpan, predInterval, numCase, numTime, trainParams.type);
disp(avgErr)

%% functions
function plot_endeffector(x,xp,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'Color',refClr,'LineWidth',2);
    hold on 
    plot(xp(:,1),xp(:,2),'r--','LineWidth',2)
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'LineWidth',2);
    end
    axis padded
    legend("Reference","Prediction","Location","best","FontName","Arial");
    title('End Effector Position')
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');
end
