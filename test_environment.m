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
tSpan = [0,5]; % [0,5] 0:0.01:5
predInterval = 5; 

%% simulation 
x0 = [0; 0; 0; 0; 0; 0]; % th0, th0d, th1, th1d, th2, th2d
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
t = y(:,1);
x = y(:,2:10);
[xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, trainParams.type);
plot_compared_states(t,x,t,xp)
disp(mean(rmseErr,'all'))

%% evaluate for four states
tSpan = [0,5];
predIntervel = 5;
numCase = 20;
numTime = 1000;
avgErr = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,1);
% avgErr = evaluate_model_with_4_states(net, sysParams, ctrlParams, trainParams, f1Max, tSpan, predInterval, numCase, numTime, trainParams.type);
disp(avgErr)
