function [vehiclesInView] = getViewedVehicles(sumo, map, outputMap, vehicleTimestep)
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
%     vehiclesInView:  Vehicles in view is a 2d array, first column is vehicle id, other
%                       columns are nearest vehicles by id. You can ignore
%                       zeros
%
% Copyright (c) 2022, Matthew Szeto
% email: maszeto@asu.edu

    global MATCHING;
    
    %Get all vehicles in the map to test, this should only be generated
    %once
    buildingsToTest = [];
    buildingIds = outputMap.buildingIncentre(:,1);
    for i = 1:length(buildingIds)
        building = outputMap.buildings(find(ismember(outputMap.buildings(:,1), buildingIds(i), 'rows')), [2,3]); 
        %building is of the form YX, so we convert to XY for use in segment
        %intersection
        buildingsToTest = [ buildingsToTest ;
            building(1:end-1,2) ...
            building(1:end-1,1) ...
            building(2:end,2)   ...
            building(2:end,1) ];
    end
    
    %Vehicles in view is a 2d array, first column is vehicle id, other
    %columns are nearest vehicles by id
    vehiclesInView = zeros(length(vehicleTimestep(:,1)), length(vehicleTimestep(:,1))+1);
    
    %distanceVehicle is a matrix representing distance of each vehicle
    %to all other vehicles in the map (vehicle id is row and column
    %index, we want the vehicle id to correlate to row and column,
    %so we need to sort vehicleTimestep first
    vehicleTimestep = sortrows(vehicleTimestep);
    distanceVehicle = pdist2(vehicleTimestep(:,2:3), vehicleTimestep(:,2:3), 'euclidean');
            
    %Check which vehicles are within LiDAR range
    vehiclesInRange = distanceVehicle<=MATCHING.lidarRad;%Checking if it is in range,

    %Now we need to fill the vehiclesInView array
    for i = 1:length(vehicleTimestep(:,1))
        %set ID
        vehiclesInView(i,1) = vehicleTimestep(i,1);
        ptr = 1;%Holds position of where last edit was made, since we are filling in arr of zeros
        for j = 1:length(vehicleTimestep(:,1))
            if (vehiclesInRange(i,j) == 1) && (i ~= j) 
                
                linkToTest = [vehicleTimestep(i,2), vehicleTimestep(i,3), vehicleTimestep(j,2), vehicleTimestep(j,3)];
                
                %Test Links, only add if there are no buildings blocking,
                %TODO: need to check for other vehicles
                intersectCnt = segments_intersect_test(linkToTest, buildingsToTest);%number of intersections
                
                if intersectCnt == 0
                    
                    %move ptr to empty index 
                    ptr = ptr + 1;
                    
                    %add vehicle ID of vehicle in view at positon
                    vehiclesInView(i, ptr) = vehicleTimestep(j,1);
                end

            end
        end
    end

end

