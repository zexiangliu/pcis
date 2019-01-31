function plot_car2(x_e,x)
%     figure(fig);
    msiz = 200; % marker size
    malpha = 0.5; % marker alpha
    fsiz = 12;
    y_e = x(2);
    h = x(3);
    car1 = scatter(x_e, y_e + 4.1, msiz, 'sb',...
            'MarkerFaceColor', 'g','MarkerEdgeColor','g');
    car2 = scatter(x_e+h, 4.1, msiz, 's',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor','r');

    text(x_e, 4.6 + y_e,'ego2 wt sp','FontSize',fsiz,...
        'Color','b','HorizontalAlignment', 'center');
    text(x_e+h, 3.8,'cau2','FontSize',fsiz,...
        'Color','k','HorizontalAlignment', 'center');

    car1.MarkerFaceAlpha = malpha;
    car2.MarkerFaceAlpha = malpha;
end