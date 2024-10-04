function params = params_training()
    params = struct();
    params.type = "dnn9"; % "dnn3","lstm3","pinn3","dnn6","lstm6","pinn6","dnn9", "lstm9","pinn9"
    params.sequenceStep = 4; % 1 for non-lstm, 4,8,16 
    params.numUnits = 8;  % number of LSTM units
    params.alpha = 0.4; % [0,1] weight of data loss and physics loss
    params.beta = 0.2;
    params.numSamples = 500; % 100,200,300,400,500
    params.numLayers = 8; % [3,10]
    params.numNeurons = 256; % 32,64,128,256
    params.dropoutFactor = 0; % 0.1,0.2
    params.miniBatchSize = 2048; % 
    params.numEpochs = 200;
    params.initTimeStep = 1;
    params.initLearningRate = 1e-3; % 0.01,0.001,0.0001
    params.stopLearningRate = 1e-7;
    params.lrDropFactor = 0.2;
    params.lrDropEpoch = 40;
end