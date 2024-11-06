function dataFile = generate_samples(sysParams, ctrlParams, trainParams)
% Generate samples and save the data file into a subfolder "data\"
    tSpan = [0,15];
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
            theta = 2*pi*rand;
            rad = sqrt(rand);
            ctrlParams.refx = ctrlParams.a*rad*cos(theta);
            ctrlParams.refy = ctrlParams.b*rad*sin(theta);
            x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
            % x0 = [0; 0; 0];
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