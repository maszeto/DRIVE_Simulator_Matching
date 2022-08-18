function [ indexPL, sortedPL ] = sortToRemoveZeros( indexPL, sortedPL )
%SORTTOREMOVEZEROS Summary of this function goes here
%   Detailed explanation goes here

    for runArray = 1:length(indexPL)
        array = nonzeros(indexPL(runArray,:))';
        array = padarray(array,[0 length(sortedPL)-length(array)],'post');
        indexPL(runArray,:) = array;

        array = nonzeros(sortedPL(runArray,:))';
        array = padarray(array,[0 length(sortedPL)-length(array)],'post');
        sortedPL(runArray,:) = array;
    end
end

