function visual(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end
    
subplot(221);hold on
    plot(V.slice([1 4], [25 25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -300 300]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 25 m/s')

    subplot(222);hold on
    plot(V.slice([1 4], [30 20]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -300 300]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s, vLead = 20 m/s')

    subplot(223);hold on
    plot(V.slice([1 4], [16 25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -300 300]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s, vLead = 25 m/s')

    subplot(224);hold on
    plot(V.slice([1 4], [25 0]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -300 300]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 0 m/s');
    drawnow;
end