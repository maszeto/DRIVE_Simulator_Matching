function [xyLinks] = getXYLinks(vehiclesStruct, matches)
%GETXYLINKS This gets the coordinates for links created from matching
% making it easier to plot
%Output is a cell array, with index representing timestep. At each
%timestep, the coordinates of the links are saved, of the form
%[x1,y1,x2,y2]
    
    for timeStep=1:length(matches)
        xyLinksCur = [];
        if ~isempty(matches{timeStep})
            for vid=1:length(matches{timeStep})
                if ~isempty(matches{timeStep}{vid})
                    for matchIdx=1:length(matches{timeStep}{vid})
                        xyLinksCur = [xyLinksCur; getVehNodePos(vid, vehiclesStruct, timeStep) ...
                            getVehNodePos(matches{timeStep}{vid}(matchIdx), vehiclesStruct, timeStep)] ;
                    end
                    
                end
            end
        end
        xyLinks{timeStep} = xyLinksCur;
        
    end


end

