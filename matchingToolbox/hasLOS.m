function LOS = hasLOS(x1,y1,x2,y2, buildingLines)
%HASLOS Check if a two points have LOS (not blocked by buildings)
%   returns boolean if there is los or not
intersectCnt = segments_intersect_test([x1,y1,x2,y2], buildingLines);%number of intersections
LOS = (intersectCnt == 0);
end

