function [a_iElements,S] = stableFixtures(vehiclesStruct, viewedVehicles, utilityFunction, plCap, timestep)
%STABLEFIXTURES Summary of this function goes here
%   Performs the stable fixtures matching game for a given timestep
%   Inputs: 
%       vehiclesStruct: Struct containing vehicle data
%       viewedVehicles: Vehicles in view, first col is vid, subsequent cols
%       are vehicle IDs in view by that veh
%       utilityFunction: Function which generates the preference list using
%                        the information from vehiclesStruct as input
%       plCap:           matching capacity for each vehicle, index is
%                        vehicleID 
%       timeStep:       Which timestep to perform the matching
%   Outputs: 
%         a_iElements: List of vehicle matches, with index being vid
%         S:           list of pairs of vehicles which are matched
        
    PL = utilityFunction(vehiclesStruct, viewedVehicles, timestep);
    
    % Sorts PL rows in descending order. 
    [ sortedPL , indexPL ] = sort(PL,2,'descend'); %Sorted PL is the utility fucntion output 
    %SortedPL is the preference value of each vehicle sorted
    %IndexPL is the sorted order of which vehicle index each vehicle prefers 

    %loop fills indexPL with 0 if sorted PL is also 0, bc that is the vehicle's
    %ranking of itself
    for i=1:length(indexPL)
        for j=1:length(indexPL)
            if sortedPL(i,j)==0
                indexPL(i,j)=0;
            end
        end
    end

    %a_iElements holds the matches for each vehicle at index i
    for i=1:length(sortedPL)
        a_iElements{i} = {};
        b_iElements{i} = {};
    end
    a_i = zeros(length(sortedPL),1); %Gives you a col of length(veh) of zeros
    S = [ 0 0 ];
    posToRun = find(a_i<plCap);%So a_i holds the number of matches for each index
    posToRun = posToRun'; %Just gives you a 1x(numveh), list of increasing numbers from 1-numVeh 

    while ~isempty(posToRun)
        [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase1( posToRun,sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements );

        [ a_i,b_i,a_iElements,b_iElements,posToRun ] = findPosToRun( plCap,indexPL,S );
    end

    S = S(2:end,:);
    if ~isempty(S)
        S = sortrows(S,1);%S contains a mapping of all pairs from a_iElements, i.e. x1 is matched to y1
    end


    [ sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements ] = sfPhase2(sortedPL,a_i,plCap,S,indexPL,a_iElements,b_iElements );

    if ~isempty(S)
        S = sortrows(S,1);
    end
end

