function[] = visualize_plots(ddt, dt, es, xs, xbars)
    t_max = length(es)*ddt;
    t = (ddt:ddt:t_max);
    te = (dt:dt:length(xbars)*dt);
    
    figure
    plot(t,es(:,1),'-r',t,es(:,2),'-g','LineWidth',1);
    xlabel('Time (s)');
    ylabel('Absolute Error (m)');
    xlim([0 t_max]);

    figure
    plot(t,xs(:,1),'-r',t,xs(:,2),'-g','LineWidth',1);
    hold on
    plot(te,xbars(:,1),'-+k',te,xbars(:,2),'-+b','LineWidth',0.5);
    legend('X','Y','X Set','Y Set');
    xlabel('Time (s)');
    ylabel('Displacement(m)');
    xlim([0 t_max]);
    hold off

    figure
    plot(t,xs(:,3),'-r',t,xs(:,4),'-g','LineWidth',1);
    hold on
    if(size(xbars,2)>2)
    plot(te,xbars(:,3),'-+k',te,xbars(:,4),'-+b','LineWidth',0.5);
        legend('Joint_1 Angle','Joint_2 Angle','Joint_1 Set','Joint_2 Set');
    else
        legend('Joint_1 Angle','Joint_2 Angle');
    end
    xlabel('Time (s)');
    ylabel('Angle(rad)');
    xlim([0 t_max]);
    hold off
end