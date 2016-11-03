%% Aquire video source
% use linux video, 1st on 'webcamlist', resolution
vid = videoinput('linuxvideo', 1, 'BGR24_640x480');
src = getselectedsource(vid);
%Aquired from 'imaqtool'
vid.FramesPerTrigger = Inf;
vid.LoggingMode = 'disk';
preview(vid);
diskLogger = VideoWriter('/home/michael/Downloads/bin/RobotX/tiggered pics/try2/hsv2_0001.avi', 'Uncompressed AVI');

%     % Acquire input video stream
%     vidDevice = imaq.VideoDevice('linuxvideo'); 
% 
%     % Set VideoFormat property
%     vidDevice.VideoFormat = 'RGB24_640x480';
% 
%     % Acquire input video property
%     vidInfo = imaqhwinfo(vidDevice); 

    % Set blob analysis handling
    hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true', ...
    'MinimumBlobArea', 200, ...
    'MaximumCount', 10);

    % Set Red box handling
    hshapeinsRedBox = vision.ShapeInserter('BorderColor', 'Custom', ... 
    'CustomBorderColor', [1 0 0], ...
    'Fill', true, ...
    'FillColor', 'Custom', ...
    'CustomFillColor', [1 0 0], ...
    'Opacity', 0.4);
    
    % Set Green box handling
    hshapeinsGreenBox = vision.ShapeInserter('BorderColor', 'Custom', ... 
    'CustomBorderColor', [0 1 0], ...
    'Fill', true, ...
    'FillColor', 'Custom', ...
    'CustomFillColor', [0 1 0], ...
    'Opacity', 0.4);

    % Output video player
    hVideoIn = vision.VideoPlayer;
    
while 1
vid.DiskLogger = diskLogger;
triggerconfig(vid, 'manual');
start(vid);
trigger(vid);    
    
    %% Processing 
    
    % Acquire single frame
    rgbFrame = vid; 
    
    % Obtain the mirror image for displaying
    rgbFrame = flip(rgbFrame,2); 
    
    % Convert RGB image to chosen color space
    hsvFrame = rgb2hsv(rgbFrame);
    
    % Define red thresholds for channel 1 based on histogram settings
    channel1RedMin = 0.920;
    channel1RedMax = 0.066;

    % Define red thresholds for channel 2 based on histogram settings
    channel2RedMin = 0.200;
    channel2RedMax = 1.000;

    % Define red thresholds for channel 3 based on histogram settings
    channel3RedMin = 0.000;
    channel3RedMax = 1.000;

    % Create red mask based on chosen histogram thresholds
    RedBW = ( (hsvFrame(:,:,1) >= channel1RedMin) | (hsvFrame(:,:,1) <= channel1RedMax) ) & ...
    (hsvFrame(:,:,2) >= channel2RedMin ) & (hsvFrame(:,:,2) <= channel2RedMax) & ...
    (hsvFrame(:,:,3) >= channel3RedMin ) & (hsvFrame(:,:,3) <= channel3RedMax);

    % Define green thresholds for channel 1 based on histogram settings
    channel1GreenMin = 0.228;
    channel1GreenMax = 0.430;

    % Define green thresholds for channel 2 based on histogram settings
    channel2GreenMin = 0.200;
    channel2GreenMax = 1.000;

    % Define green thresholds for channel 3 based on histogram settings
    channel3GreenMin = 0.000;
    channel3GreenMax = 1.000;

    % Create green mask based on chosen histogram thresholds
    GreenBW = (hsvFrame(:,:,1) >= channel1GreenMin ) & (hsvFrame(:,:,1) <= channel1GreenMax) & ...
    (hsvFrame(:,:,2) >= channel2GreenMin ) & (hsvFrame(:,:,2) <= channel2GreenMax) & ...
    (hsvFrame(:,:,3) >= channel3GreenMin ) & (hsvFrame(:,:,3) <= channel3GreenMax);
    
    % Get the centroids and bounding boxes of the red blobs
    [centroidRed, bboxRed] = step(hblob, RedBW); 
    
    % Convert the centroids into Integer for further steps
    centroidRed = uint16(centroidRed); 
    
    % Get the centroids and bounding boxes of the green blobs
    [centroidGreen, bboxGreen] = step(hblob, GreenBW); 

    % Convert the centroids into Integer for further steps
    centroidGreen = uint16(centroidGreen); 
    
    % Instert the red box
    vidIn = step(hshapeinsRedBox, rgbFrame, bboxRed); 
    
    % Instert the green box
    vidIn = step(hshapeinsGreenBox, vidIn, bboxGreen); 

    % Output video stream
    step(hVideoIn, vidIn); 

end

%% Clearing Memory

    % Release all memory and buffer used
    release(hVideoIn); 
    release(vidDevice);
    imaqreset;
