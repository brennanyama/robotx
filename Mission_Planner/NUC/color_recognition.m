function [centroidRed, angleRed] = color_recognition(vidDevice)
    % Create structural element for morphological operations to remove
    % disturbances
    rectangleElem = strel('rectangle',[4 1]);
    
    % Create a BlobAnalysis object to calculate detected objects' area,
    % centroid, and bounding box
    hblob = vision.BlobAnalysis('AreaOutputPort', true, ... 
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true', ...
    'MinimumBlobArea', 200, ...
    'MaximumBlobArea', 5000);

    % Set Red box handling
    hshapeinsRedBox = vision.ShapeInserter('BorderColor', 'Custom', ... 
    'CustomBorderColor', [1 0 0], ...
    'Fill', true, ...
    'FillColor', 'Custom', ...
    'CustomFillColor', [1 0 0], ...
    'Opacity', 0.4);
    
    % Acquire single frame
    rgbFrame = step(vidDevice);

    % Convert RGB image to chosen color space
    hsvFrame = rgb2hsv(rgbFrame);

    % Define red thresholds for channel 1 based on histogram settings
    channel1RedMin = 0.916;
    channel1RedMax = 0.039;

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

    % Remove disturbances using morphological operations
    RedBWopen = imopen(RedBW,rectangleElem);

    % Get the centroids and bounding boxes of the red blobs
    [areaRed,centroidRed,bboxRed] = step(hblob,RedBWopen);

    % Convert the centroids into Integer for further steps
%     centroidRed = uint16(centroidRed);
    
    % Calculate Angle
    if(~isempty(centroidRed))
        angleRed = 90-atan((centroidRed(1, 2)-480)/(centroidRed(1,1)-320));
    else 
        angleRed = 0;
    end
end