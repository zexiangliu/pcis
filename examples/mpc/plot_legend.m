clc;close all;
fig = figure('position',[100 100 440 400]);
hold on;
msiz = 200; % marker size
fsiz = 12;
% car1 = scatter(0, 1.3, msiz, 'sb',...
%             'MarkerFaceColor', 'b','MarkerEdgeColor','b');
% car2 = scatter(0, 1.2, msiz, 's',...
%             'MarkerFaceColor', 'k', 'MarkerEdgeColor','k');
% car3 = scatter(0, 1.1, msiz, 'd',...
%         'MarkerFaceColor', 'g','MarkerEdgeColor','g');
% car4 = scatter(0, 1, msiz, 'd',...
%     'MarkerFaceColor', 'r','MarkerEdgeColor','r');

% car1.MarkerFaceAlpha = malpha;
% car2.MarkerFaceAlpha = malpha;
% car3.MarkerFaceAlpha = malpha;
% car4.MarkerFaceAlpha = malpha;
im1 = imread('cars/car_blue.png');
im2 = imread('cars/car_black.png');
im3 = imread('cars/car_green.png');
im4 = imread('cars/car_red.png');
len = 0.05;
wid = 0.03;
x1 = [0-len,0+len];
y1 = [1.3-wid,1.3+wid];
x2 = [0-len,0+len];
y2 = [1.2-wid,1.2+wid];
x3 = [0-len,0+len];
y3 = [1.1-wid,1.1+wid];
x4 = [0-len,0+len];
y4 = [1.0-wid,1.0+wid];
car1 = imagesc(x1, y1,im1);
car2 = imagesc(x2, y2,im2);
car3 = imagesc(x3, y3,im3);
car4 = imagesc(x4, y4,im4);


axis off;
bias = 0.15;
text(bias, 1.3,'ego car 1 without supervisor','FontSize',fsiz,...
    'Color','b','HorizontalAlignment', 'left');
text(bias, 1.2,'aggressive driver 1','FontSize',fsiz,...
    'Color','k','HorizontalAlignment', 'left');

text(bias, 1.1,'ego car 2 with supervisor','FontSize',fsiz,...
    'Color','g','HorizontalAlignment', 'left');
text(bias, 1, 'aggressive driver 2','FontSize',fsiz, ...
    'Color','r','HorizontalAlignment', 'left');

plot(0,0.9,'og','markersize',15, 'linewidth',2);
plot(0,0.8,'.g','markersize',30);

text(bias, 0.9,'user selected input','FontSize',fsiz,...
    'Color','g','HorizontalAlignment', 'left');
text(bias, 0.8, 'overriding input','FontSize',fsiz, ...
    'Color','g','HorizontalAlignment', 'left');
axis tight;

V = [-0.1 0.7;
    -0.1 1.4;
    0.7 1.4;
    0.7 0.7];
F = [1 2 3 4];
patch('Faces',F,'Vertices',V,'FaceColor','none')
print(fig,'./pic/legend','-dpng');
