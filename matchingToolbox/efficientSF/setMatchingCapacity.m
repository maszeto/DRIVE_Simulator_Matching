function [ matchingCapacity ] = setMatchingCapacity( matchingCapacity,vehicleStruct )
%SETMATCHINGCAPACITY This function takes the matching capacity declared in
%the simulation setting and creates an array with length equal to the
%number of vehicles
%
% Input:
%   matchingCapacity  : This is the maximum capacity for the matching game
%   vehicleStruct     : This structure contains all the information about the vehicles
%                       such as the real position, the GPS position, the beacons, etc.
% Output:
%   matchingCapacity  : The array with length equal to the number of
%                       vehicles is returned
%
% Copyright (c) 2016-2017, Ioannis Mavromatis

    [ capSizeX, ~, ~ ] = size(matchingCapacity);
    if capSizeX~=vehicleStruct.numNodes
        matchingCapacity(1:vehicleStruct.numNodes,1) = matchingCapacity;
    end
end

