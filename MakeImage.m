function MakeImage(ctrlParams,sysParams, t, x, xp, ref, tSpan)
    idx = find(t <= tSpan(2), 1, 'last');
    Xcg = x(idx,1);
    Xcg_pred = xp(idx,1);
    [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(x(idx,1:3),sysParams);
    [~,~,~,~,xpend1,ypend1,xpend2,ypend2] = ForwardKinematics(xp(idx,1:3),sysParams);
    
    idx0 = find(t <= tSpan(1), 1, 'last');
    Xcg0 = x(idx0,1);
    [~,~,~,~,x0end1,y0end1,x0end2,y0end2] = ForwardKinematics(x(idx0,1:3),sysParams);

    % Reference
    
    % Animation
    Ycg = 0;
    % plot limits
    Xmin = -7;
    Xmax = 7;
    Ymin = -3;
    Ymax = 3;
    cartHalfLen = 0.4;
    
    f = figure('Color', 'White');
    f.Position = [500 200 800 500];
    hold on
    % Plot one frame...
    [h1,h2,h3,h5]=robot_plot_frame(ctrlParams,Xcg_pred,Ycg,cartHalfLen,Xcg,xend1,yend1,xend2,yend2,xpend1,ypend1,xpend2,ypend2,ref);

    % System initial state
    patch('XData', Xcg0+[-cartHalfLen cartHalfLen cartHalfLen -cartHalfLen],...
        'YData', Ycg+[cartHalfLen cartHalfLen -cartHalfLen -cartHalfLen],...
        'FaceColor','none', 'FaceAlpha', 0, ...
        'EdgeColor','k','LineWidth',1,'LineStyle','--');
    % plots pendulum
    h4 = plot([Xcg0 x0end1],[Ycg y0end1],'k','LineWidth', 1, 'LineStyle','--', "DisplayName", "Initial Position"); 
    plot(x0end1,y0end1,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k'); 
    plot([x0end1 x0end2],[y0end1 y0end2],'k','LineWidth', 1, 'LineStyle','--'); 
    plot(x0end2,y0end2,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k');

    disErr = Xcg_pred - Xcg;
    angErr1 = x(idx,2) - xp(idx,2);
    angErr2 = x(idx,3) - xp(idx,3);
    annotation('textbox', [0.8, 0.53, 0.12, 0.1], ...
        'String', {"$\theta_0$ error: "+num2str(disErr,'%.3f') + " m" , "$\theta_1$  error: " + num2str(angErr1,'%.3f') + " rad", "$\theta_2$ error: " + num2str(angErr2,'%.3f') + " rad"}, ...
        'FitBoxToText', 'on', ...
        'BackgroundColor', 'white', ...
        'EdgeColor', 'White', ...
        'FontName', 'Arial', ...
        'FontSize', 11, ...
        "Interpreter","latex");
    
    axis([Xmin Xmax Ymin Ymax])
    set(gca, "FontName", "Arial");
    set(gca, "FontSize", 12);
    xlabel("(m)", "FontSize", 15, "FontName","Arial")
    ylabel("(m)", "FontSize", 15, "FontName","Arial")
    daspect([1 1 1])

    tObj = title("System at "+num2str(tSpan(2)-tSpan(1))+" second", "FontName", "Arial","FontSize",15);
    tObj.Position(1) = -3.0;
    legend([h1 h2 h3 h4 h5], "FontName","Arial", "FontSize", 10, 'Position', [0.85, 0.7, 0.04, 0.05]);
    
    saveas(f,'robot_image.jpg')
end
