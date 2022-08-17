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
%     vehiclesInView: Struct mapping Vehicle IDs to an array of vehicles it
%                       can see
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
        buildingsToTest = [ buildingsToTest ;
            building(1:end-1,1) ...
            building(1:end-1,2) ...
            building(2:end,1)   ...
            building(2:end,2) ];
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
    
    %linksToTest = [0,0,0,0];
    
    %Now we need to fill the vehiclesInView array
    for i = 1:length(vehicleTimestep(:,1))
        %set ID
        vehiclesInView(i,1) = vehicleTimestep(i,1) + 1;
        ptr = 1;%Holds position of where last edit was made, since we are filling in arr of zeros
        for j = 1:length(vehicleTimestep(:,1))
            if (vehiclesInRange(i,j) == 1) && (i ~= j) 
                %move ptr to empty index 
                ptr = ptr + 1;
                
                linkToTest = [vehicleTimestep(i,2), vehicleTimestep(i,3), vehicleTimestep(j,2), vehicleTimestep(j,3)];
                
                %Test Links
                intersectCnt = segments_intersect_test(linkToTest, buildingsToTest);
                
                    %add vehicle ID of vehicle in view at positon
                    vehiclesInView(i, ptr) = vehicleTimestep(j,1) + 1;
                
                
                
                %now add the pair of points to linksToTest in the form
                %[x1,y1,x2,y2] check to make sure [x2,y2,x1,y1] is not
                %already in the array TODO:Optimize
%                 if(ismember(linksToTest, [vehicleTimestep(j,2), ...
%                         vehicleTimestep(j,3), vehicleTimestep(i,2), ...
%                         vehicleTimestep(i,3)], 'rows') == 0)
%                     linksToTest = [linksToTest; [vehicleTimestep(i,2), vehicleTimestep(i,3), vehicleTimestep(j,2), vehicleTimestep(j,3)]]; 
%                     %test link
%                     
%                 end
            end
        end
    end
%     linksToTest(1,:) = [];%remove zero row from init
%     %Now that we have vehicles in range of one another, we need to test LoS
%     %Links to test
%     %remove duplicates in links to test, we have recorded each v2v link
%     %twice
% %     [~,iULinks,~] = unique(sort(linksToTest,2), 'rows');%Find indeces of unique links
% %     uLinksToTest = linksToTest(iULinks,:);
%     
%     %raysToTest = [ repmat(outputMap.inCentresTile(1,1:2),[length(sortedIndexes{1}),1]), outputMap.inCentresTile(sortedIndexes{1},1:2) ];
%     %So rayst to test is in the [x1,y1,x2,y2] format, we can replace rays
%     %with p2p v2v links
% 
%     
%     interSect = segments_intersect_test_vector(linksToTest,buildingsToTest);
% 
%     %  Find the intersections with the road polygons
%     interSect = logical(interSect);
%     for k = 1:length(buildingIds{1}) %Not sure how vehicle IDs is decided
%         building = [ outputMap.buildings( ismember(outputMap.buildings(:,1), buildingIds{1}(k)), 3 ) ...
%             outputMap.buildings( ismember(outputMap.buildings(:,1), buildingIds{1}(k)), 2 ) ]; 
%         % I think this is all the points in the perimeter of the building ^
%         %This creates short vectors between perimeter points of the
%         %building, [x1,y1,x1+1,y1+1] (not +1 but one index more)
%         buildingsToTest = [ buildingsToTest ;
%             building(1:end-1,1) ...
%             building(1:end-1,2) ...
%             building(2:end,1)   ...
%             building(2:end,2) ];
%     end
%     if ~isempty(buildingsToTest)
%         % if any NaNs were parsed during the loading phase of SUMO or
%         % while loading the OSM map, remove these buildings
%         buildingsToTest(isnan(buildingsToTest(:,1)),:) = [];
%         buildingsToTest(isnan(buildingsToTest(:,3)),:) = [];
% 
%         % calculate the LOS and NLOS status for the all links
%         % for each given ray using the buildings found before
%         [ losLinks,nLosLinks, losIDs{1}, nLosIDs{1},losNlosStatus{1} ] = ...
%             losNlosCalculation(sortedIndexes{1},raysToTest,buildingsToTest);
% 
%     else
%         % when no buildings are available, all tiles are assumed to
%         % be in LOS
%         losIDs{1} = sortedIndexes{1};
%         losNlosStatus{1} = true(1,length(losIDs{1}));
%     end
    
    
    
    
    
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

