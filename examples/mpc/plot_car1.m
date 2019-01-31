function plot_car1(x_e,x)
%     figure(fig);
    msiz = 200; % marker size
    malpha = 0.5; % marker alpha
    fsiz = 12;
    y_e = x(2);
    h = x(3);
    car1 = scatter(x_e, y_e + 4.1, msiz, 'sb',...
            'MarkerFaceColor', 'b','MarkerEdgeColor','b');
    car2 = scatter(x_e+h, 4.1, msiz, 's',...
        'MarkerFaceColor', 'k', 'MarkerEdgeColor','k');

    text(x_e, 5.5 + y_e,'ego1 wo sp','FontSize',fsiz,...
        'Color','b','HorizontalAlignment', 'center');
    text(x_e+h, 4.3,'cau1','FontSize',fsiz,...
        'Color','k','HorizontalAlignment', 'center');

%         car1.MarkerFaceAlpha = malpha;
    car2.MarkerFaceAlpha = malpha;
        
end