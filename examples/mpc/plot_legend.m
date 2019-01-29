clc;close all;
fig = figure('position',[100 100 440 400]);
hold on;
msiz = 200; % marker size
fsiz = 12;
car1 = scatter(0, 1.3, msiz, 'sb',...
            'MarkerFaceColor', 'b','MarkerEdgeColor','b');
car2 = scatter(0, 1.2, msiz, 's',...
            'MarkerFaceColor', 'k', 'MarkerEdgeColor','k');
car3 = scatter(0, 1.1, msiz, 'd',...
        'MarkerFaceColor', 'g','MarkerEdgeColor','g');
car4 = scatter(0, 1, msiz, 'd',...
    'MarkerFaceColor', 'r','MarkerEdgeColor','r');
car1.MarkerFaceAlpha = malpha;
car2.MarkerFaceAlpha = malpha;
car3.MarkerFaceAlpha = malpha;
car4.MarkerFaceAlpha = malpha;
axis off;

text(0.2, 1.3,'ego car 1 without supervisor','FontSize',fsiz,...
    'Color','b','HorizontalAlignment', 'left');
text(0.2, 1.2,'cautious driver 1','FontSize',fsiz,...
    'Color','k','HorizontalAlignment', 'left');

text(0.2, 1.1,'ego car 2 with supervisor','FontSize',fsiz,...
    'Color','g','HorizontalAlignment', 'left');
text(0.2, 1, 'cautious driver 2','FontSize',fsiz, ...
    'Color','r','HorizontalAlignment', 'left');

plot(0,0.9,'og','markersize',15, 'linewidth',2);
plot(0,0.8,'.g','markersize',30);

text(0.2, 0.9,'user selected input','FontSize',fsiz,...
    'Color','g','HorizontalAlignment', 'left');
text(0.2, 0.8, 'overriding input','FontSize',fsiz, ...
    'Color','g','HorizontalAlignment', 'left');
axis tight;

V = [-0.1 0.7;
    -0.1 1.4;
    0.7 1.4;
    0.7 0.7];
F = [1 2 3 4];
patch('Faces',F,'Vertices',V,'FaceColor','none')
print(fig,'./pic/legend','-dpng');
