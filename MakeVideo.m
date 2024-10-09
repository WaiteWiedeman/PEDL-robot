function MakeVideo(sysParams, t, x, xp, tSpan)
     % Set up video
    v=VideoWriter('robot_animation.avi');
    v.FrameRate=30;
    open(v);

    idx1 = find(t <= tSpan(1), 1, 'last');
    idx2 = find(t <= tSpan(2), 1, 'last');

    % Animation
    Ycg = 0;
    % plot limits
    Xmin = -5;
    Xmax = 5;
    Ymin = -1;
    Ymax = 2;
    cartHalfLen = 0.7;
   
    f = figure('Color', 'White');
    f.Position = [500 100 800 900];

    for n = idx1:idx2
        cla
        Xcg = x(n,1);
        Xcg_pred = xp(n,1);
        [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(x(n,1:3),sysParams);
        [~,~,~,~,xpend1,ypend1,xpend2,ypend2] = ForwardKinematics(xp(n,1:3),sysParams);
        
        subplot(4,1,1)
        plot(t(idx1:n)-1,x(idx1:n,1),'k-',t(idx1:n)-1,xp(idx1:n,1),'r--','LineWidth',2);
        set(gca, 'FontSize', 12); % Set font size of ticks
        ylabel('$\theta_0$',"Interpreter","latex", 'FontSize', 18);
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontName', "Arial")
        axis([0,max(t)-1 min(xp(:,1))-1 max(xp(:,1))+1])
        set(gca,'Position',[0.1,0.8,0.8,0.15]);
        grid on
        
        subplot(4,1,2)
        plot(t(idx1:n)-1,x(idx1:n,2),'k-',t(idx1:n)-1,xp(idx1:n,2),'r--','LineWidth',2);
        set(gca, 'FontSize', 12); % Set font size of ticks
        ylabel('$\theta_1$',"Interpreter","latex", 'FontSize', 18);
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontName', "Arial")
        axis([0,max(t)-1 min(xp(:,2))-1 max(xp(:,2))+1])
        set(gca,'Position',[0.1,0.6,0.8,0.15]);
        xlabel("Time (s)",'FontSize', 15);
        set(gca, 'FontSize', 12); % Set font size of ticks
        grid on;

        subplot(4,1,3)
        plot(t(idx1:n)-1,x(idx1:n,3),'k-',t(idx1:n)-1,xp(idx1:n,3),'r--','LineWidth',2);
        set(gca, 'FontSize', 12); % Set font size of ticks
        ylabel('$\theta_2$',"Interpreter","latex", 'FontSize', 18);
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontName', "Arial")
        axis([0,max(t)-1 min(xp(:,3))-1 max(xp(:,3))+1])
        set(gca,'Position',[0.1,0.4,0.8,0.15]);
        xlabel("Time (s)",'FontSize', 15);
        set(gca, 'FontSize', 12); % Set font size of ticks
        grid on;
        
        subplot(4,1,4)
        hold on
        % Plot one frame...
        [h1,h2] = robot_plot_frame(Xcg_pred,Ycg,cartHalfLen,Xcg,xend1,yend1,xend2,yend2,xpend1,ypend1,xpend2,ypend2);

        disErr = Xcg_pred - Xcg;
        angErr1 = x(n,2) - xp(n,2);
        angErr2 = x(n,3) - xp(n,3);
        annotation('textbox', [0.12, 0.0, 0.3, 0.2], ...
            'String', {"$\theta_0$ error: "+num2str(disErr,'%.3f') + " m" , "$\theta_1$  error: " + num2str(angErr1,'%.3f') + " rad", "$\theta_2$ error: " + num2str(angErr2,'%.3f') + " rad"}, ...
            'FitBoxToText', 'on', ...
            'BackgroundColor', 'white', ...
            'EdgeColor', 'White', ...
            'FontName', 'Arial', ...
            'FontSize', 15, ...
            "Interpreter","latex");

        axis([Xmin Xmax Ymin Ymax])
        set(gca, "YTick", []);
        set(gca, "FontName", "Arial");
        set(gca, "FontSize", 12);
        xlabel("(m)", "FontSize", 15, "FontName","Arial")
        daspect([1 1 1])
        set(gca,'Position',[0.1,0.0,0.8,0.4]);

        tObj = title("System at "+num2str(tSpan(2)-tSpan(1))+" second", "FontName", "Arial","FontSize",15);
        tObj.Position(1) = -3.0;
        legend([h1 h2], "FontName","Arial", "FontSize", 15, 'Position', [0.13, 0.2, 0.2, 0.1]);

        frame=getframe(gcf);
        writeVideo(v,frame);
    end
end