function output = train_pinn_model_9(sampleFile, trainParams,sysParams,ctrlParams,monitor)
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
                if length(xGroup(startIdx:endIdx)) < 10
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
    disp(num2str(length(cell2mat(xTrain))) + " samples are generated for training.");
    
    % Create neural network
    numStates = 9;
    layers = [
        featureInputLayer(numStates+1, "Name", "input")
        ];
    
    numMiddle = floor(trainParams.numLayers/2);
    for i = 1:numMiddle
        layers = [
            layers
            fullyConnectedLayer(trainParams.numNeurons)
            eluLayer
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
            eluLayer
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
                    "TestAccuracy"...
                    "ExecutionEnvironment"];
    monitor.XLabel = "Iteration";
    
    net = train_adam_update(net, xTrain, yTrain, trainParams, monitor);
    output.trainedNet = net;

    ctrlParams.solver = "stiffhr"; % "stiff" or "normal"
    ctrlParams.method = "origin"; % random, interval, origin
    tSpan = [0,20];
    predInterval = tSpan(2);
    numCase = 50;
    numTime = 500;
    initTime = 1;
    updateInfo(monitor,...
        TestAccuracy=evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,0, initTime));
end

%%
function net = train_adam_update(net, xTrain, yTrain, trainParams, monitor)
    % using stochastic gradient decent
    miniBatchSize = trainParams.miniBatchSize/trainParams.nmPts;
    lrRate = trainParams.initLearningRate;
    dataSize = length(xTrain);
    numBatches = floor(dataSize/miniBatchSize);
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

    accFcn = dlaccelerate(@modelLoss);
    
    avgGrad = [];
    avgSqGrad = [];
    iter = 0;
    epoch = 0;
    while epoch < trainParams.numEpochs && ~monitor.Stop
        epoch = epoch + 1;
        % Shuffle data.
        shuffle(mbq);

        while hasdata(mbq) && ~monitor.Stop
            iter = iter + 1;

            % Read mini-batch of data.
            [X,T] = next(mbq);

            % Evaluate the model loss and gradients using dlfeval and the
            % modelLoss function.
            [loss, gradients] = dlfeval(accFcn, net, X, T);
            
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

            executionEnvironment = "auto";

            if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
                updateInfo(monitor,ExecutionEnvironment="GPU");
            else
                updateInfo(monitor,ExecutionEnvironment="CPU");
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
function [loss, gradients] = modelLoss(net, xBatch, tBatch)
    % Get parameters
    sysParams = params_system();
    trainParams = params_training();

    % Split inputs and targets into cell arrays
    % xBatch = {};
    % tBatch = {};

    [zBatch, ~] = forward(net, xBatch);
    dataLoss = mse(zBatch, tBatch);

    xBatch = extractdata(xBatch);
    tBatch = extractdata(tBatch);
    zBatch = extractdata(zBatch);

    i = 2; % intialize index
    while ~isempty(xBatch)
        if xBatch(:,i) ~= xBatch(:,i-1)
            % xBatch = [xBatch X(:,1:i-1)];
            % tBatch = [tBatch T(:,1:i-1)];
            % X(:,1:i-1) = [];
            % T(:,1:i-1) = [];
            X = xBatch(1:9,1:i-1); % initial states
            T = xBatch(10,1:i-1); % times
            Y = tBatch(:,1:i-1); % targets
            Z = zBatch(:,1:i-1); % prediction

            % compute gradients using automatic differentiation
            q1 = Z(1,:);
            q2 = Z(2,:);
            q3 = Z(3,:);
            q1d = Z(4,:);
            q2d = Z(5,:);
            q3d = Z(6,:);
            q1dd = Z(7,:);
            q2dd = Z(8,:);
            q3dd = Z(9,:);
            % velocities calulated as gradient of position prediction
            q1dn = gradient(extractdata(q1),T);
            q2dn = gradient(extractdata(q2),T);
            q3dn = gradient(extractdata(q3),T);
            % accelerations calulated as gradient of velocity prediction
            q1ddn = gradient(extractdata(q1d),T);
            q2ddn = gradient(extractdata(q2d),T);
            q3ddn = gradient(extractdata(q3d),T);

            q1ddn = gpuArray(dlarray(q1ddn, "CB"));
            q2ddn = gpuArray(dlarray(q2ddn, "CB"));
            q3ddn = gpuArray(dlarray(q3ddn, "CB"));
            q1dn = gpuArray(dlarray(q1dn, "CB"));
            q2dn = gpuArray(dlarray(q2dn, "CB"));
            q3dn = gpuArray(dlarray(q3dn, "CB"));

            fY = physics_law([q1;q2;q3;(0.5*q1d+0.5*q1dn);(0.5*q2d+0.5*q2dn);(0.5*q3d+0.5*q3dn);...
                (0.5*q1dd+0.5*q1ddn);(0.5*q2dd+0.5*q2ddn);(0.5*q3dd+0.5*q3ddn)],sysParams);
            fT = physics_law(Y,sysParams);
            physicLoss(i) = mse(fY, fT);

            [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(transpose(extractdata(Z(1:3,:))),sysParams);
            [~,~,~,~,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(transpose(Y(1:3,:)),sysParams);

            xend1 = gpuArray(dlarray(transpose(xend1), "CB"));
            yend1 = gpuArray(dlarray(transpose(yend1), "CB"));
            xend2 = gpuArray(dlarray(transpose(xend2), "CB"));
            yend2 = gpuArray(dlarray(transpose(yend2), "CB"));
            xendTarget1 = gpuArray(dlarray(transpose(xendTarget1), "CB"));
            yendTarget1 = gpuArray(dlarray(transpose(yendTarget1), "CB"));
            xendTarget2 = gpuArray(dlarray(transpose(xendTarget2), "CB"));
            yendTarget2 = gpuArray(dlarray(transpose(yendTarget2), "CB"));

            endEff = [xend1;yend1;xend2;yend2];
            endEffTarget = [xendTarget1;yendTarget1;xendTarget2;yendTarget2];
            endEffloss(i) = mse(endEff,endEffTarget);

            i = 2;
        elseif i == length(xBatch)
            xBatch = [xBatch X];
            tBatch = [tBatch T];
            X = [];
            T = [];
        else
            i = i + 1;
        end
    end
    % total loss
    loss = (1.0-trainParams.alpha-trainParams.beta)*sum(dataLoss,"all") + trainParams.alpha*sum(physicLoss,"all") + trainParams.beta*sum(endEffloss,"all");
    
    gradients = dlgradient(loss, net.Learnables);
    % % calculate loss for each group collocation points
    % dataLoss = gpuArray(dlarray(zeros(length(tBatch),1), "CB"));
    % physicLoss = dataLoss;
    % endEffloss = dataLoss;
    % for i = 1:length(tBatch)
    %     T = xBatch{i}(10,:);
    %     X = xBatch{i}(1:9,:);
    %     Y = tBatch{i};
    % 
    %     % make prediction
    %     [Z, ~] = forward(net, [dlarray(X,"CB");dlarray(T,"CB")]);
    %     dataLoss(i) = mse(Z, Y);
    % 
    %     % compute gradients using automatic differentiation
    %     q1 = Z(1,:);
    %     q2 = Z(2,:);
    %     q3 = Z(3,:);
    %     q1d = Z(4,:);
    %     q2d = Z(5,:);
    %     q3d = Z(6,:);
    %     q1dd = Z(7,:);
    %     q2dd = Z(8,:);
    %     q3dd = Z(9,:);
    % 
    %     % velocities calulated as gradient of position prediction
    %     q1dn = gradient(extractdata(q1),T);
    %     q2dn = gradient(extractdata(q2),T);
    %     q3dn = gradient(extractdata(q3),T);
    %     % accelerations calulated as gradient of velocity prediction
    %     q1ddn = gradient(extractdata(q1d),T);
    %     q2ddn = gradient(extractdata(q2d),T);
    %     q3ddn = gradient(extractdata(q3d),T);
    % 
    %     q1ddn = gpuArray(dlarray(q1ddn, "CB"));
    %     q2ddn = gpuArray(dlarray(q2ddn, "CB"));
    %     q3ddn = gpuArray(dlarray(q3ddn, "CB"));
    %     q1dn = gpuArray(dlarray(q1dn, "CB"));
    %     q2dn = gpuArray(dlarray(q2dn, "CB"));
    %     q3dn = gpuArray(dlarray(q3dn, "CB"));
    %     % fY = gpuArray(dlarray(zeros(3,length(T)), "CB"));
    %     % fT = fY;
    %     % for j = 1:length(T)
    %     %     fY(:,j) = physics_law([q1(j);q2(j);q3(j);(0.5*q1d(j)+0.5*q1dn(j));(0.5*q2d(j)+0.5*q2dn(j));(0.5*q3d(j)+0.5*q3dn(j));...
    %     %         q1dd(j);q2dd(j);q3dd(j)],sysParams);
    %     %     fT(:,j) = physics_law(Y(:,j),sysParams);
    %     % end
    %     fY = physics_law([q1;q2;q3;(0.5*q1d+0.5*q1dn);(0.5*q2d+0.5*q2dn);(0.5*q3d+0.5*q3dn);...
    %             (0.5*q1dd+0.5*q1ddn);(0.5*q2dd+0.5*q2ddn);(0.5*q3dd+0.5*q3ddn)],sysParams);
    %     fT = physics_law(Y,sysParams);
    %     % disp(fY)
    %     % disp(fT)
    %     physicLoss(i) = mse(fY, fT);
    % 
    %     % End Effector loss
    %     % % xend1 = gpuArray(dlarray(zeros(length(T),1), "CB"));
    %     % % yend1 = xend1;
    %     % % xend2 = xend1;
    %     % % yend2 = xend1;
    %     % % xendTarget1 = xend1;
    %     % % yendTarget1 = xend1;
    %     % % xendTarget2 = xend1;
    %     % % yendTarget2 = xend1;
    %     % % endEff = gpuArray(dlarray(zeros(4,length(T)), "CB"));
    %     % % endEffTarget = endEff;
    %     % 
    %     % % for j = 1:length(T)
    %     % %     [~,~,~,~,xend1(j),yend1(j),xend2(j),yend2(j)] = ForwardKinematics(Z(1:3,j),sysParams);
    %     % %     [~,~,~,~,xendTarget1(j),yendTarget1(j),xendTarget2(j),yendTarget2(j)] = ForwardKinematics(Y(1:3,j),sysParams);
    %     % %     % endEff(:,j) = [xend1(j);yend1(j);xend2(j);yend2(j)];
    %     % %     % endEffTarget(:,j) = [xendTarget1(j);yendTarget1(j);xendTarget2(j);yendTarget2(j)];
    %     % % end
    %     [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(transpose(extractdata(Z(1:3,:))),sysParams);
    %     [~,~,~,~,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(transpose(Y(1:3,:)),sysParams);
    % 
    %     xend1 = gpuArray(dlarray(transpose(xend1), "CB"));
    %     yend1 = gpuArray(dlarray(transpose(yend1), "CB"));
    %     xend2 = gpuArray(dlarray(transpose(xend2), "CB"));
    %     yend2 = gpuArray(dlarray(transpose(yend2), "CB"));
    %     xendTarget1 = gpuArray(dlarray(transpose(xendTarget1), "CB"));
    %     yendTarget1 = gpuArray(dlarray(transpose(yendTarget1), "CB"));
    %     xendTarget2 = gpuArray(dlarray(transpose(xendTarget2), "CB"));
    %     yendTarget2 = gpuArray(dlarray(transpose(yendTarget2), "CB"));
    % 
    %     endEff = [xend1;yend1;xend2;yend2];
    %     endEffTarget = [xendTarget1;yendTarget1;xendTarget2;yendTarget2];
    %     endEffloss(i) = mse(endEff,endEffTarget);
    % end
    % % total loss
    % loss = (1.0-trainParams.alpha-trainParams.beta)*sum(dataLoss,"all") + trainParams.alpha*sum(physicLoss,"all") + trainParams.beta*sum(endEffloss,"all");
    % 
    % gradients = dlgradient(loss, net.Learnables);
    
end

function [X,T] = myMiniBatch(xBatch,yBatch)
    X = [];
    T = [];
    for i = 1:length(xBatch)
        X = [X, cell2mat(xBatch{i})];
        T = [T, cell2mat(yBatch{i})];
    end
end
    