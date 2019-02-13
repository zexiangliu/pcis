mkdir new_pic

for i = 0:150
    lege = imread('./pic/legend.png');
    frame = imread(['./pic/frame',num2str(i),'.png']);

    lege = lege(:,1:end,:);
    siz1 = size(lege);
    % imshow(lege)
    siz2 = size(frame);
    %%
    shift = [70,-100];
    lege = im2double(lege);
    frame = im2double(frame);
    new_frame = ones(siz2(1)+1,siz1(2)+siz2(2),3);
    new_frame(1:siz2(1),1:siz2(2),:) = frame;
    new_frame(1+shift(1):siz1(1)+shift(1),siz2(2)+1+shift(2):siz2(2)+siz1(2)+shift(2),:) = lege;
    imwrite(new_frame,sprintf('./new_pic/frame%03d.png',i));
end