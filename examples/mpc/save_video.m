function save_video(frame,filename)
SIZ = zeros(length(frame),3);
for i = 1:length(frame)
    SIZ(i,:) = size(frame(i).cdata);
end
siz = min(SIZ);

for i = 1:length(frame)
    frame(i).cdata = frame(i).cdata(1:siz(1),1:siz(2),1:siz(3));
end

video = VideoWriter(filename);
video.FrameRate = 10;
video.Quality = 100;
open(video);
writeVideo(video,frame);
close(video);
end