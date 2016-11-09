%read the sop. make sure you fix diskLogger path, rename every time.
%imaqreset whenever video is stuck on. blahblahblah
imaqreset
vid = videoinput('linuxvideo', 1, 'RGB24_640x480');
src = getselectedsource(vid);

vid.FramesPerTrigger = Inf;
vid.LoggingMode = 'disk';
diskLogger = VideoWriter('/media/michael/FRL/camerastuff/lookatmenow6_0001.avi', 'Uncompressed AVI');
vid.DiskLogger = diskLogger;
triggerconfig(vid, 'manual');
preview(vid);

while 1
start(vid);

trigger(vid);
end

stoppreview(vid);
stop(vid);