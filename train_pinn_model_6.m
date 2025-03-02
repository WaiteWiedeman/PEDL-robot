function output = train_pinn_model_6(sampleFile, trainParams,sysParams,ctrlParams,monitor)
% PINN
% A physics-Informed Neural Network (PINN) is a type of neural network
% architecture desigend to incorporate physical principles or equations
% into the learning process. In combines deep learning techniques with
% domain-specific knowledge, making it particularly suitable for problems
% governed by physics.
% In addition to standard data-driven training, PINNs utilize terms in the
% loss function to enforce consistency with know physical law, equations,
% and constraints. 
% https://en.wikipedia.org/wiki/Physics-informed_neural_networks 
% https://benmoseley.blog/my-research/so-what-is-a-physics-informed-neural-network/
    
    %initialize output
    output.trainedNet = [];

    % load samples and prepare training dataset
    ds = load(sampleFile);
    numSamples = trainParams.numSamples;   
    
    % generate data
    % Feature data: 6-D initial state x0 + time interval
    % the label data is a predicted state x=[q1,q2,q1dot,q2dot,q1ddot,q2ddot]
    initTimes = 1:trainParams.initTimeStep:4; %start from 1 sec to 4 sec with 0.5 sec step 
    tTrain = [];
    xTrain = [];
    yTrain = [];
    for i = 1:numSamples
        data = load(ds.samples{i,1}).state;
        t = data(1,:);
        x = data(2:7, :); % q1,q2,q1_dot,q2_dot
        for tInit = initTimes
            initIdx = find(t > tInit, 1, 'first');
            x0 = x(:, initIdx); % Initial state 
            t0 = t(initIdx); % Start time
            for j = initIdx+1 : length(t)
                tTrain = [tTrain, t(j)-t0];
                xTrain = [xTrain, x0];
                yTrain = [yTrain, x(:,j)];
            end
        end
    end
    disp(num2str(length(xTrain)) + " samples are generated for training.");
    
    % Create neural network
    numStates = 6;
    layers = [
        featureInputLayer(numStates+1, "Name", "input")
        ];
    
    numMiddle = floor(trainParams.numLayers/2);
    for i = 1:numMiddle
        layers = [
            layers
            fullyConnectedLayer(trainParams.numNeurons)
            tanhLayer
        ];
    end
    if trainParams.dropoutFactor > 0
        layers = [
            layers
            dropoutLayer(trainParams.dropoutFactor)
        ];
    end
    for i = numMiddle+1:trainParams.numLayers
        layers = [
            layers
            fullyConnectedLayer(trainParams.numNeurons)
            tanhLayer
        ];
    end
    
    layers = [
        layers
        fullyConnectedLayer(numStates, "Name", "output")
       ];

    % convert the layer array to a dlnetwork object
    net = dlnetwork(layers);
    net = dlupdate(@double, net);
    % plot(net)
    
    % training options
    monitor.Metrics = "Loss";
    monitor.Info = ["LearnRate" ... 
                    "IterationPerEpoch" ...
                    "MaximumIteration" ...
                    "Epoch" ...
                    "Iteration" ...
                    "GradientsNorm"...
                    "StepNorm"...
                    "TestAccuracy"];
    monitor.XLabel = "Iteration";
    
    net = train_adam_update(net, tTrain, xTrain, yTrain, trainParams, monitor);
    output.trainedNet = net;

    f1Max = [5,30];
    tSpan = [0,5];
    predIntervel = 5;
    numCase = 2;
    numTime = 100;
    updateInfo(monitor,...
        TestAccuracy=evaluate_model(net, sysParams, ctrlParams, trainParams, f1Max, tSpan, predIntervel, numCase, numTime, trainParams.type));
end

%%
function net = train_adam_update(net, tTrain, xTrain, yTrain, trainParams, monitor)
    % using stochastic gradient decent
    miniBatchSize = trainParams.miniBatchSize;
    lrRate = trainParams.initLearningRate;
    dataSize = length(tTrain);
    numBatches = floor(dataSize/miniBatchSize);
    numIterations = trainParams.numEpochs * numBatches;

    accFcn = dlaccelerate(@modelLoss);
    
    avgGrad = [];
    avgSqGrad = [];
    iter = 0;
    epoch = 0;
    while epoch < trainParams.numEpochs && ~monitor.Stop
        epoch = epoch + 1;
        % Shuffle data.
        idx = randperm(dataSize);
        tAll = tTrain(:, idx);
        XAll = xTrain(:, idx);
        YAll = yTrain(:, idx);
        for j = 1 : numBatches
            iter = iter + 1;
            startIdx = (j-1)*miniBatchSize + 1;
            endIdx = min(j*miniBatchSize, dataSize);
            tBatch = tAll(:, startIdx:endIdx);
            xBatch = XAll(:, startIdx:endIdx);
            yBatch = YAll(:, startIdx:endIdx); 
            T = gpuArray(dlarray(tBatch, "CB"));
            X = gpuArray(dlarray(xBatch, "CB"));
            Y = gpuArray(dlarray(yBatch, "CB"));
            % Evaluate the model loss and gradients using dlfeval and the
            % modelLoss function.
            [loss, gradients] = dlfeval(accFcn, net, T, X, Y);
    
            % Update the network parameters using the ADAM optimizer.
            [net, avgGrad, avgSqGrad] = adamupdate(net, gradients, avgGrad, avgSqGrad, iter, lrRate);
    
            recordMetrics(monitor, iter, Loss=loss);
    
            if mod(iter, trainParams.numEpochs) == 0
                monitor.Progress = 100*iter/numIterations;
                updateInfo(monitor, ...
                    LearnRate = lrRate, ...
                    Epoch = epoch, ...
                    Iteration = iter, ...
                    MaximumIteration = numIterations, ...
                    IterationPerEpoch = numBatches);
            end
        end
        % adaptive learning rate
        if mod(epoch,trainParams.lrDropEpoch) == 0
            if lrRate > trainParams.stopLearningRate
                lrRate = lrRate*trainParams.lrDropFactor;
            else
                lrRate = trainParams.stopLearningRate;
            end
        end
    end
end

%% loss function
function [loss, gradients] = modelLoss(net, T, X, Y)
    % make prediction
    [Z, ~] = forward(net, [X;T]);
    dataLoss = mse(Z, Y);

    % compute gradients using automatic differentiation
    q1 = Z(1,:);
    q2 = Z(2,:);
    q1d = Z(3,:);
    q2d = Z(4,:);
    q1dd = dlgradient(sum(q1d, 'all'), T);
    q2dd = dlgradient(sum(q2d, 'all'), T);
    % q1d = dlgradient(sum(q1, 'all'), T);
    % q2d = dlgradient(sum(q2, 'all'), T);
    
    fY = physics_law([q1;q2], [q1d;q2d], [q1dd;q2dd]);
    fT = zeros(size(fY), 'like', fY);
    physicLoss = mse(fY, fT);
    
    % total loss
    trainParams = params_training();
    loss = (1.0-trainParams.alpha)*dataLoss + trainParams.alpha*(physicLoss);
    gradients = dlgradient(loss, net.Learnables);
end
