function [centroidRed, angleRed] = color_recognition(vidDevice)
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
    centroidRed = uint16(centroidRed);
    
    % Calculate Angle
    angleRed = 90-atan((centroidRed(1, 2)-480)/(centroidRed(1,1)-320));
end