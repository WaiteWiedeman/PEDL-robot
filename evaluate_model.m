function avgErr = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, type, show, initTime)
    % evaluate time span, larger time span will increase the simulation
    % time when complicated friction involved
    th0 = linspace(-1,1,numCase);
    th1 = linspace(-2*pi,2*pi,numCase);

    % reference time points 
    switch trainParams.type
        case {"dnn3", "lstm3", "pinn3", "pirn3"} 
            errs = zeros(3*numCase, numTime);
        case {"dnn6", "lstm6", "pinn6", "pirn6"} 
            errs = zeros(6*numCase, numTime);
        case {"dnn9", "lstm9", "pinn9", "pirn9"} 
            errs = zeros(9*numCase, numTime);
        otherwise
            disp("unspecify type of model.")
    end
    for i = 1:numCase
        x0 = [th0(i); 0; th1(i); 0; th1(i); 0]; % th0, th0d, th1, th1d, th2, th2d
        y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
        t = y(:,1);
        x = y(:,2:10);
        [xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, type, initTime);
        if show
            disp("evaluate " + num2str(i) + " th case, mean square err: " + num2str(mean(rmseErr, "all")));
        end
        switch trainParams.type
            case {"dnn3", "lstm3", "pinn3", "pirn3"} 
                errs(3*(i-1)+1:3*(i-1)+3,:) = rmseErr;
            case {"dnn6", "lstm6", "pinn6", "pirn6"} 
                errs(6*(i-1)+1:6*(i-1)+6,:) = rmseErr;
            case {"dnn9", "lstm9", "pinn9", "pirn9"} 
                errs(9*(i-1)+1:9*(i-1)+9,:) = rmseErr;    
            otherwise
                disp("unspecify type of model.")
        end
    end
    
    avgErr = mean(errs,'all'); % one value of error for estimtation
    if show
        disp("plot time step rsme")
        figure('Position',[500,100,800,300]); 
        tiledlayout("vertical","TileSpacing","tight")
        plot(refTime,mean(errs,1),'k-','LineWidth',2);
        xlabel("Time (s)","FontName","Arial");
        ylabel("Average RMSE","FontName","Arial");
        xticks(linspace(1,tSpan(2),(tSpan(2))));
        title("Average RMSE: "+num2str(avgErr));
        set(gca, 'FontSize', 15);
    end
end



