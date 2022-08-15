function [vehiclesInView] = getViewedVehicles(sumo,map, outputMap, vehicleTimestep)
%GETVIEWEDVEHICLES Get vehicles in LiDAR LOS from input vehicle 
%   For the current timestep, return a list of Vehicle ID's of each vehicle
%   within the LiDAR LOS for every vehicle in the sim 
%   Output Row index corresponds to vid and columns are the vehicles in
%   that vehicle's LOS, the vehicles in view are candidates for the
%   matching game
%  Input  :
%     sumo          : Structure containing all the SUMO settings (maximum
%                     number of vehicles, start time, end time, etc.)
%     map           : Structure containing all map settings (filename, path,
%                     simplification tolerance to be used, etc).
%     outputMap     : The map structure containing all the map information.
%    
%     vehicleTimestep: is all the vehicles positions at the current
%                      timestep (ID, X, Y, timestep, id) (given from
%                      getVehiclesAndPedestrians function)
%
%  Output :
%     vehiclesInView: Struct mapping Vehicle IDs to an array of vehicles it
%                       can see
%
% Copyright (c) 2022, Matthew Szeto
% email: maszeto@asu.edu

    global MATCHING;


    vehiclesInView.vid = [];%Array of vehicleIDs
    %vehiclesInView.viewed = [];%Array of arrays of viewed vehicles;
    
    %distanceVehicle is a matrix representing distance of each vehicle
    %to all other vehicles in the map (vehicle id is row and column
    %index, we want the vehicle id to correlate to row and column,
    %so we need to sort vehicleTimestep first
    vehicleTimestep = sortrows(vehicleTimestep);
    distanceVehicle = pdist2(vehicleTimestep(:,2:3), vehicleTimestep(:,2:3), 'euclidean');
            
    %Check which vehicles are within LiDAR range
    vehiclesInRange = distanceVehicle<=MATCHING.lidarRad;%Checking if it is in range,
            
    %TODO Then check LoS
    
    %Build vehiclesInView struct 
    
%     for i = 1:size(vehiclesInRange:1)
%         
%         %Get this vehicles ID
%         vehiclesInView.vid(i) = vehicleTimestep(i,1);
%         
%         %use temp array to get list of vehicles in view
%         tmp = [];
%         for j = 1:size(1:vehiclesInRange)
%             if vehiclesInRange(i,j) == 1 && i~=j 
%                 tmp(end+1)= vehicleTimestep(j,1);
%             end
%         end
%         
%         %set temp array 
%         vehiclesInView.veiwed(i) = tmp;
%         
%     end
%     10 - 1;
end

