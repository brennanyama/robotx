%Note: shows same display in two seperate windows, but it is taking data
%from the two different inputs. Red object and blob appear in window 1 from
%camera 1. Green blob appears in window 2 from camera 2. I don't know know
%why but because we don't need to view it as it is happening, I don't care
%to fix that problem.

%% Initialization

% Acquire input video stream
vidDevice = imaq.VideoDevice('linuxvideo',1);         %webcam1
vidDevice2 = imaq.VideoDevice('linuxvideo',2);        %webcam2

% Set VideoFormat property
vidDevice.VideoFormat = 'RGB24_640x480';              %webcam1
vidDevice2.VideoFormat = 'RGB24_640x480';             %webcam2

% Acquire input video property
% vidInfo = imaqhwinfo(vidDevice);

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
hVideoIn2 = vision.VideoPlayer;

while 1                    %'while' processing
    %% Processing
    
    % Acquire single frame
    rgbFrame = step(vidDevice);         %webcam1
    rgbFrame2 = step(vidDevice2);         %webcam2
    
    % Convert RGB image to chosen color space
    hsvFrame = rgb2hsv(rgbFrame);       %webcam1
        hsvFrame2 = rgb2hsv(rgbFrame2);       %webcam2

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
    GreenBW = (hsvFrame2(:,:,1) >= channel1GreenMin ) & (hsvFrame2(:,:,1) <= channel1GreenMax) & ...
        (hsvFrame2(:,:,2) >= channel2GreenMin ) & (hsvFrame2(:,:,2) <= channel2GreenMax) & ...
        (hsvFrame2(:,:,3) >= channel3GreenMin ) & (hsvFrame2(:,:,3) <= channel3GreenMax);
    
    % Get the centroids and bounding boxes of the red blobs
    [centroidRed, bboxRed] = step(hblob, RedBW);
    
    % Get the centroids and bounding boxes of the green blobs
    [centroidGreen, bboxGreen] = step(hblob, GreenBW);
    
    % Instert the red box
    vidIn = step(hshapeinsRedBox, rgbFrame, bboxRed);
    
    % Instert the green box
    vidIn2 = step(hshapeinsGreenBox, rgbFrame, bboxGreen);
    
    % Output video stream
    step(hVideoIn, vidIn);              %webcam1
    step(hVideoIn2, vidIn2);             %webcam2
    
    %% To calculate a minium area to be regestered
    wr = bboxRed(:,3);           %to get width
    hr = bboxRed(:,4);           %to get height
    Ar = wr.*hr;                   %calculate area
    wg = bboxGreen(:,3);
    hg = bboxGreen(:,4);
    Ag = wg.*hg;
    if Ar>2000
        %% To find Heading
        %get the x coord; expanding rows of coloumn 1
        xr = centroidRed(:,1);
        %get the y coord; expanding rows of coulumn 2
        yr = centroidRed(:,2);
        %0<=x<320; if the centroid is on the left
        if 0<=xr<320
            ax=320-xr;               %half way subtract x coord
            ay=480-yr;               %because (0,0) is topmost left corner
            alpha=atan2(ay,ax);     %solve arctan
        else 321<xr<=640
            ax=320+xr;               %half way add the x coord
            ay=480-yr;               %becasue (0,0) is topmost left corner
            alpha=atan2(ay,ax);     %solve arctan
        end                         %'if/else'
        alpha=alpha*180/pi;         %changes alpha from rad to deg
        alpha = alpha-90;           %makes 90degree forward
        red = 1;
        disp(red)
        disp(centroidRed)
        disp(alpha)                 %displays the alpha value
    end                       %end if Ar>XXXX
    if Ag>2000
        %% To find Heading Green
        %get the x coord; expanding rows of coloumn 1
        xg = centroidGreen(:,1);
        %get the y coord; expanding rows of coulumn 2
        yg = centroidGreen(:,2);
        %0<=x<320; if the centroid is on the left
        if 0<=xg<320
            bx=320-xg;               %half way subtract x coord
            by=480-yg;               %because (0,0) is topmost left corner
            beta=atan2(by,bx);     %solve arctan
        else 321<xg<=640
            bx=320+xg;               %half way add the x coord
            by=480-yg;               %becasue (0,0) is topmost left corner
            beta=atan2(by,bx);     %solve arctan
        end                         %'if/else'
        beta=beta*180/pi;         %changes alpha from rad to deg
        beta = beta-90;           %makes 90degree forward
        green=2;
        disp(green)
        disp(centroidGreen)
        disp(beta)                 %displays the alpha value
    end                        %end ifAg>xxx
    
    
end                            %'while' processig

%% Clearing Memory

% Release all memory and buffer used
release(hVideoIn);
release(vidDevice);
