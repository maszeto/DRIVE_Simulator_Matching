function [xyLinks] = getXYLinks(vehiclesStruct, matches)
%GETXYLINKS This gets the coordinates for links created from matching
% making it easier to plot
%Output is a cell array, with index representing timestep. At each
%timestep, the coordinates of the links are saved, of the form
%[x1,y1,x2,y2]
    
    for timeStep=1:length(matches)
        xyLinksCur = [];
        if ~isempty(matches{timeStep})
            for vehicleIndex=1:length(matches{timeStep})
                if ~isempty(matches{timeStep}{vehicleIndex})
                    for matchIdx=1:length(matches{timeStep}{vehicleIndex})
                        vid = vehiclesStruct.vehNode(vehicleIndex).id;
                        matchedVid = vehiclesStruct.vehNode(matches{timeStep}{vehicleIndex}(matchIdx)).id;
                        xyLinksCur = [xyLinksCur; getVehNodePos(vid, vehiclesStruct, timeStep) ...
                            getVehNodePos(matchedVid, vehiclesStruct, timeStep)] ;
                    end
                    
                end
            end
        end
        xyLinks{timeStep} = xyLinksCur;
        
    end


end

