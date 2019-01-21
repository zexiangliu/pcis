function visual3(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end
    
subplot(221);hold on
    plot(V.slice([1], [25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s')

    subplot(222);hold on
    plot(V.slice([1], [30]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s')

    subplot(223);hold on
    plot(V.slice([1], [16]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s')

    subplot(224);hold on
    plot(V.slice([1], [25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s');
    drawnow;
end