

% total rsus is the sum of potential rsus at each timestep 
potRSUs = {
    [1,2,3,4,5],
    [1,2,3,4],
    [3,4,6,7],
    [4,5],
    [5],
    [5,6,7],
    [6,7],
    [8]
    };
cellsz = cellfun(@size,A,'uni',false); 
%define adjacency matrix

names = [];
for i = 1:50
    names = [names, num2str(i) + "_" + num2str(idivide(int16(i),5))];
end
vDAG = digraph(A, names);

plot(vDAG,'Layout','force');

cellsz = cellfun(@size,A,'uni',false);
