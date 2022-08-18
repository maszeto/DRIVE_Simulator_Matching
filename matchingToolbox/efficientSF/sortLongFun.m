function [sortLong, indexLong ] = sortLongFun(d,indexPL)
%SORTLONG Summary of this function goes here
%   Detailed explanation goes here

    for i = 1:length(d)
        tmpSum = sum(indexPL(i,:)~=0);
        sortLong(i) = d(i)<=tmpSum;
    end
    for i = 1:length(indexPL)
        longBool(i) = d(i,1) < nnz(indexPL(i,:));
    end

    indexLong = find(longBool==1);
    if any(d==0)
        tmpIndex = find(d==0);
        sortLong(tmpIndex)=1;
    end

end

