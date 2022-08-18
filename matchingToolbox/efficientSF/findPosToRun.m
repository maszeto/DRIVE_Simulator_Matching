function [ a_i,b_i,a_iElements,b_iElements,posToRun ] = findPosToRun( plCap,indexPL,S )
%FINDPOSTORUN Summary of this function goes here
%   Detailed explanation goes here

    for j = 1:length(indexPL)
        a_i(j,1) = sum(S(:,1)==j);
        b_i(j,1) = sum(S(:,2)==j);

        aPos = find(S(:,1)==j);
        bPos = find(S(:,2)==j);
        a_iElements{j} = S(aPos,2);
        b_iElements{j} = S(bPos,1);
    end
    
    posToRun = find( a_i < min( plCap, nnz(indexPL(:,:)) ));
    posToRunTmp = posToRun;
    for index = 1:numel(posToRun)
        if nnz(indexPL(posToRun(index),:))==0 || nnz(indexPL(posToRun(index),:))<=a_i(posToRun(index))
            posToRunTmp(index) = 0;
        end
    end
    posToRun = nonzeros(posToRunTmp)';

end

