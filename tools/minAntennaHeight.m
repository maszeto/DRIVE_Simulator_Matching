function [minAntennaHeight] = minAntennaHeight(carHeight, truckHeight, carLaneNum, laneWidth, rsuDist)
%MINANTENNAHEIGHT This function calculates the minimum required antenna
%height based on a paper
%   Detailed explanation goes here
    halfLaneWidth = laneWidth / 2;
    minAntennaHeight = carHeight + ((rsuDist + (carLaneNum-1)*laneWidth + halfLaneWidth) ...
                        / halfLaneWidth)*(truckHeight - carHeight);
end

