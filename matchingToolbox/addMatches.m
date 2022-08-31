function [vehiclesStruct] = addMatches(matches,vehiclesStruct)
%ADDMATCHES Adds vehicles matched to vehicle struct 
%   Modifies the vehicle struct created from parseMobility. At each
%   timestep, every vehicle now has an array of vehicles it has within it's
%   been matched to. 
%
%  Input  :
%     matches      : 1x(simTime) cell array, each index represents
%                   timestep. Each value is a 1x(vehNodeLength) cell array.
%                   Where index represents vehicle ID and value is an array
%                   of matched vehicle IDs. 
%     vehiclesStruct : A structure containing all the information about the
%                       vehicles.
%  Output :
%     vehiclesStruct : A structure containing all the information about the
%                       vehicles. Now with a new field "Matches"
for idx = 1:length(vehiclesStruct.vehNode)
    
    vehiclesStruct.vehNode(idx).matches = cell([length(vehiclesStruct.vehNode(idx).time) 1]);%init cell arr of size time
    
    for j = 1:length(vehiclesStruct.vehNode(idx).time)
        
        timeStep = vehiclesStruct.vehNode(idx).time(j);
        
        %Get the array of which vehicles are available at this timestep
        if ~isempty(matches{timeStep})
            curMatches = matches{timeStep}{idx}';
        else
            curMatches = [];
        end
        
        vehiclesStruct.vehNode(idx).matches{j} = curMatches;
               
    end
    
    
    
end

