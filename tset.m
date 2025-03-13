clc; clear; close all;

%% params
ctrlParams = params_control();
sysParams = params_system(); 
trainParams = params_training();

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

%% Cell array manipulation
% a = 1:200;
% dataSize = length(a);
% nmPts = 64;
% nmGrps = ceil(dataSize/nmPts);
% b = cell(nmGrps,1);
% for i = 1:nmGrps
%     startIdx = (i-1)*nmPts + 1;
%     endIdx = min(i*nmPts, dataSize);
%     b{i} = a(startIdx:endIdx);
% end    

sampleFile = "trainingSamples.mat";
ds = load(sampleFile);
numSamples = 100;

% generate data
% Feature data: 6-D initial state x0 + time interval
% the label data is a predicted state x=[q1,q2,q1dot,q2dot,q1ddot,q2ddot]
initTimes = 1:trainParams.initTimeStep:6; %start from 1 sec to 4 sec with 0.5 sec step
% tGroup = [];
xGroup = [];
yGroup = [];
% tTrain = {};
xTrain = {};
yTrain = {};
for i = 1:numSamples
    data = load(ds.samples{i,1}).state;
    t = data(1,:);
    x = data(2:10, :); % q1,q2,q1_dot,q2_dot
    for tInit = initTimes
        initIdx = find(t > tInit, 1, 'first');
        x0 = x(:, initIdx); % Initial state
        t0 = t(initIdx); % Start time
        for j = initIdx+1 : length(t)
            % tGroup = [tGroup, t(j)-t0];
            xGroup = [xGroup, [x0; t(j)-t0]];
            yGroup = [yGroup, x(:,j)];
        end
        dataSize = length(xGroup);
        nmGrps = ceil(dataSize/trainParams.nmPts);
        for z = 1:nmGrps
            startIdx = (z-1)*trainParams.nmPts + 1;
            endIdx = min(z*trainParams.nmPts, dataSize);
            if length(xGroup(startIdx:endIdx)) < (trainParams.nmPts/5)
                xTrain(end) = {[cell2mat(xTrain(end)) xGroup(:,startIdx:endIdx)]};
                yTrain(end) = {[cell2mat(yTrain(end)) yGroup(:,startIdx:endIdx)]};
            else
                % tTrain = [tTrain tGroup(startIdx:endIdx)];
                xTrain = [xTrain xGroup(:,startIdx:endIdx)];
                yTrain = [yTrain yGroup(:,startIdx:endIdx)];
            end
        end
        % tGroup = [];
        xGroup = [];
        yGroup = [];
    end
end

miniBatchSize = trainParams.miniBatchSize/trainParams.nmPts;
dataSize = length(xTrain);
numBatches = ceil(dataSize/miniBatchSize);
numIterations = trainParams.numEpochs * numBatches;

dsState = arrayDatastore(xTrain, "ReadSize", miniBatchSize,"IterationDimension",2); %"OutputType", "same", 
dsLabel = arrayDatastore(yTrain, "ReadSize", miniBatchSize,"IterationDimension",2);
dsTrain = combine(dsState, dsLabel); 

mbq = minibatchqueue(dsTrain,...
    MiniBatchSize=miniBatchSize, ...
    MiniBatchFormat="CB", ... 
    MiniBatchFcn=@myMiniBatch, ...
    OutputEnvironment="gpu", ...
    PartialMiniBatch="discard");

shuffle(mbq)
[X,T] = next(mbq);

% xBatch = {};
% tBatch = {};
X = extractdata(X);
T = extractdata(T);

ids = find(diff(X(1,:)) ~= 0);
sz = size(X);
% startIds = [ids+1];
boundaryIds = [ids+1 ids sz(2)];

q1d = gradient(T(1,:),X(10,:)); %zeros(1,sz(2));

q1dtrue = T(4,:);

q1d(boundaryIds) = [];
q1dtrue(boundaryIds) = [];

% for i = 1:length(startIds)
%     td = X(10,startIds(i):endIds(i));
%     y = T(1,startIds(i):endIds(i));
%     q1d(startIds(i):endIds(i)) = gradient(y,td);
% end
gradErr = q1dtrue - q1d;
% i = 2;
% while ~isempty(X)
%     if X(:,i) ~= X(:,i-1)
%         xBatch = [xBatch X(:,1:i-1)];
%         tBatch = [tBatch T(:,1:i-1)];
%         X(:,1:i-1) = [];
%         T(:,1:i-1) = [];
%         i = 2;
%     elseif i == length(X)
%         xBatch = [xBatch X];
%         tBatch = [tBatch T];
%         X = [];
%         T = [];
%     else
%         i = i + 1;
%     end
% end

% iter = 0;
% epoch = 0;
% trainParams.numEpochs = 1;
% while epoch < trainParams.numEpochs 
%     epoch = epoch + 1;
%     % Shuffle data.
%     idx = randperm(dataSize);
%     xTrainShffld = xTrain(idx);
%     yTrainShffld = yTrain(idx);
%     for j = 1 : numBatches
%         iter = iter + 1;
%         startIdx = (j-1)*miniBatchSize + 1;
%         endIdx = min(j*miniBatchSize, dataSize);
%         xBatch = xTrainShffld(startIdx:endIdx);
%         yBatch = yTrainShffld(startIdx:endIdx);
%     end
% end

function [X,T] = myMiniBatch(xBatch,yBatch)
    X = [];
    T = [];
    for i = 1:length(xBatch)
        X = [X, cell2mat(xBatch{i})];
        T = [T, cell2mat(yBatch{i})];
    end
    % disp(xBatch)
    % X = cat(2,xBatch{:});
    % disp(X)
    % T = cat(2,yBatch{:});
end
