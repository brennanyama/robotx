%% Initialization

%     Threshold for red detection
    redThresh = 0.24; 

%     Threshold for green detection
    greenThresh = 0.05; 
    
    % Acquire input video stream
    vidDevice = imaq.VideoDevice('linuxvideo'); 
    
    % Set VideoFormat property
    vidDevice.VideoFormat = 'RGB24_640x480';
    
    % Acquire input video property
    vidInfo = imaqhwinfo(vidDevice); 

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
%% Processing 
    
    % Acquire single frame
    rgbFrame = step(vidDevice); 
    
    % Obtain the mirror image for displaying
    rgbFrame = flip(rgbFrame,2); 
    
%     Get red component of the image
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame)); 
    
%     Filter out the noise by using median filter
    diffFrameRed = medfilt2(diffFrameRed, [3 3]); 
    
%     Convert the image into binary image with the red objects as white
    binFrameRed = imbinarize(diffFrameRed, redThresh); 
    
%     Get green component of the image
    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame)); 
    
%     Filter out the noise by using median filter
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]); 
    
%     Convert the image into binary image with the green objects as white
    binFrameGreen = imbinarize(diffFrameGreen, greenThresh); 
    
    % Get the centroids and bounding boxes of the red blobs
    [centroidRed, bboxRed] = step(hblob, binFrameRed); 
    
    % Convert the centroids into Integer for further steps
    centroidRed = uint16(centroidRed); 
    
    % Get the centroids and bounding boxes of the green blobs
    [centroidGreen, bboxGreen] = step(hblob, binFrameGreen); 

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
         