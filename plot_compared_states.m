function plot_compared_states(t,x,tp,xp,flag,refs)
labels= ["$\theta_0$","$\theta_1$","$\theta_2$","$\dot{\theta}_0$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
    "$\ddot{\theta}_0$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
switch flag
    case "position"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 1:numState-6
            nexttile
            plot(t,x(:,i),'b-',tp,xp(:,i),'r--',t,refs(:,i),'k:','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            if i == 1
                ylabel(labels(i) + "[m]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad]","Interpreter","latex");
            end
            % set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-6
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Prediction","Reference","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 4:numState-3
            nexttile
            plot(t,x(:,i),'b-',tp,xp(:,i),'r--','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            if i == 4
                ylabel(labels(i) + "[m/s]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad/s]","Interpreter","latex");
            end
            % set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-3
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 7:numState
            nexttile
            plot(t,x(:,i),'b-',tp,xp(:,i),'r--','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            if i == 7
                ylabel(labels(i) + "[m/s/s]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad/s/s]","Interpreter","latex");
            end
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end