function pos = convM2P(xG, yG, binaryMap)

    resolution = binaryMap.Resolution;
    
    x = (xG-513)/resolution;
    y = (yG-512)/resolution;
    pos = [x, y];
end