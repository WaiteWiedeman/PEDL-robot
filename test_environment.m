%% clear workspace
close all; clear; clc;

%% test variables
file = "best_dnn4_models_3";
%file = "best_pinn6_models";
net = load(file).dnn4_200_50_5;
% net = load(file).pinn6_256_6_800.trainedNet;
sysParams = params_system();
ctrlParams = params_control();
trainParams = params_training();
trainParams.type = "dnn4"; % "dnn4","lstm4","pinn4","dnn6", "lstm6","pinn6"
ctrlParams.method = "interval";
numTime = 100;
F1 = 45;
tSpan = [0,10];
predInterval = 10;

%% simulation
f1Max = F1; 
ctrlParams.fMax = [f1Max; 0];
y = sdpm_simulation(tSpan, sysParams, ctrlParams);
t = y(:,1);
x = y(:,2:7);
[xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, trainParams.type);
plot_compared_states(t,x,t,xp)
disp(mean(rmseErr,'all'))
