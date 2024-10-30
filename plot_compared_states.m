function plot_compared_states(t,x,tp,xp)
    labels= ["$\theta_0$","$\theta_1$","$\theta_2$","$\dot{\theta}_0$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
        "$\ddot{\theta}_0$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
    figure('Position',[500,200,600,600]);
    tiledlayout("vertical","TileSpacing","tight")
    numState = size(xp);
    numState = numState(2);
    for i = 1:numState
        nexttile
        plot(t,x(:,i),'b-',tp,xp(:,i),'r--','LineWidth',2);
        hold on
        xline(1,'k--', 'LineWidth',1);
        ylabel(labels(i),"Interpreter","latex");
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontSize', 15);
        set(gca, 'FontName', "Arial")
        if i == numState
            xlabel("Time (s)");
        end
    end 
    % legend("Reference","Prediction","Location","eastoutside","FontName","Arial");
end