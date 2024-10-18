function dataFile = generate_samples(sysParams, ctrlParams, trainParams)
% Generate samples and save the data file into a subfolder "data\"
    tSpan = [0,5];
    dataFile = "trainingSamples.mat";
    % check whether need to regenerate samples
    regenerate_samples = 1; % by default, regrenerate samples
    if exist(dataFile, 'file') == 2
        ds = load(dataFile);
        if trainParams.numSamples == length(ds.samples)
            regenerate_samples = 0;
        end
    end
    
    % generate sample data
    if regenerate_samples      
        samples = {};
        for i = 1:trainParams.numSamples
            %disp("generate data for " + num2str(i) + "th sample.");
            ctrlParams.PID0(1) = 5000 + 20000*rand;
            ctrlParams.PID1(1) = 5000 + 20000*rand;
            ctrlParams.PID2(1) = 5000 + 20000*rand;
            x0 = [-1; -2*pi; -2*pi] + [2; 4*pi; 4*pi].*rand(3,1); % th0, th1, th2
            x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
            y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
            state = y';
            fname=['data\input',num2str(i),'.mat'];
            save(fname, 'state');
            samples{end+1} = fname;
        end
        samples = reshape(samples, [], 1); % make it row-based
        save(dataFile, 'samples');
    else
        %disp(num2str(trainParams.numSamples) + " samples is already generated.");
    end
end