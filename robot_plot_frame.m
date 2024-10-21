function [h1,h2,h3]=robot_plot_frame(Xcg_pred,Ycg,cartHalfLen,Xcg,xend1,yend1,xend2,yend2,xpend1,ypend1,xpend2,ypend2,ref)
    % True system
    % cart
    patch(Xcg+[-cartHalfLen cartHalfLen cartHalfLen -cartHalfLen], ...
        Ycg+[cartHalfLen cartHalfLen -cartHalfLen -cartHalfLen],'k','FaceAlpha', 0, 'LineWidth',2); 
    
    % plots rod and blob
    h1 = plot([Xcg xend1],[Ycg yend1],'k','LineWidth', 3, 'LineStyle','-', "DisplayName", "Ground Truth"); 
    plot(xend1,yend1,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k'); 
    plot([xend1 xend2],[yend1 yend2],'k','LineWidth', 3, 'LineStyle','-'); 
    plot(xend2,yend2,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k');
    
    % System predicted by model
    % cart
    patch('XData', Xcg_pred+[-cartHalfLen cartHalfLen cartHalfLen -cartHalfLen],...
        'YData', Ycg+[cartHalfLen cartHalfLen -cartHalfLen -cartHalfLen],...
        'FaceColor','none', 'FaceAlpha', 0, ...
        'EdgeColor','r','LineWidth',2,'LineStyle','--');

    % plots pendulum
    h2 = plot([Xcg_pred xpend1],[Ycg ypend1],'r','LineWidth', 2, 'LineStyle','--', "DisplayName", "Prediction"); 
    plot(xpend1,ypend1,'Marker','o','MarkerSize',12,'MarkerEdgeColor','r'); 
    plot([xpend1 xpend2],[ypend1 ypend2],'r','LineWidth', 2, 'LineStyle','--'); 
    plot(xpend2,ypend2,'Marker','o','MarkerSize',12,'MarkerEdgeColor','r');

    % plots reference being tracked by arm
    Xd = ref(:,1);
    Yd = ref(:,2);
    h3 = plot(Xd,Yd,'k','LineWidth', 1, 'LineStyle',':', "DisplayName", "Reference Trajectory");
end