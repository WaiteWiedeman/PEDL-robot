function avgErr = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, type)
    % evaluate time span, larger time span will increase the simulation
    % time when complicated friction involved
    
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
        y = robot_simulation(tSpan, sysParams, ctrlParams);
        t = y(:,1);
        x = y(:,2:10);
        [xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, type);
        %disp("evaluate "+num2str(i)+" th case, f1: "+num2str(f1Max) + " N, mean square err: " + num2str(mean(rmseErr, "all")));
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

    % disp("plot time step rsme")
    % figure('Position',[500,100,800,300]); 
    % tiledlayout("vertical","TileSpacing","tight")
    % plot(refTime,mean(errs,1),'k-','LineWidth',2);
    % xlabel("Time (s)","FontName","Arial");
    % ylabel("Average RMSE","FontName","Arial");
    % xticks(linspace(1,tSpan(2),(tSpan(2))));
    % title("Average RMSE: "+num2str(avgErr));
    % set(gca, 'FontSize', 15);
end



