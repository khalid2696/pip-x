% The MIT License (MIT)
%
% Copyright June, 2019 Michael Otte, University of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% The software is provided "as is", without warranty of any kind, express or
% implied, including but not limited to the warranties of merchantability,
% fitness for a particular purpose and noninfringement. In no event shall the
% authors or copyright holders be liable for any claim, damages or other
% liability, whether in an action of contract, tort or otherwise, arising from,
% out of or in connection with the software or the use or other dealings in
% the software.

classdef ghostPointIterator < handle

    % this helps with KDTree dimensions that wrap around
    
    % Copyright (c) January, 2014 Michael Otte, MIT (original Julia Version)
    % Copyright 2019 Michael Otte, University of Maryland (Matlab Version)
     
        
    properties
          kdTree             % the kd tree hold that this is being used with
          queryPoint         % Array{Float64}, the actual point that all the ghosts are "identical" with

          wrapDimFlags       % Array{Int}, wrapDimFlags indicates the current permutation
                             % of all ghost points that we are currently looking at

          ghostTreeDepth     % a pointer to the current depth of the "gost tree"
                             % note that this "tree" is only a theoretical construct that
                             % determines the order in which the ghosts are returned
                             % it should not be confused with the kdTree

          currentGhost       % Array{Float64}, the current ghost that we are returning

          closestUnwrappedPoint   % Array{Float64}, the closest point in the normal space to the
                                  % currentGhost, dist between this and ghost can
                                  % be used as uristic to skip unhelpfull ghosts


    end
    methods
        % constructor
        function obj = ghostPointIterator(kdTree, queryPoint)
            obj.kdTree = kdTree;
            obj.queryPoint = queryPoint;
            obj.wrapDimFlags = zeros(kdTree.numWraps,1);
            obj.ghostTreeDepth = kdTree.numWraps;
            obj.currentGhost = queryPoint;
            obj.closestUnwrappedPoint = queryPoint;
        end
        
        
        
        % this returns the next ghost point, note it starts at the first -ghost- and does
        % not return the original point
        function ret = getNextGhostPoint(G, bestDist)

            while true % will return out when done
                % go up tree until we find a wrapDimFlags[ghostTreeDepth] == 0 (this 
                % indicates that we need to try permutations where ghost is wrapped 
                % around tree.wraps[ghostTreeDepth]

                while G.ghostTreeDepth > 0 && G.wrapDimFlags(G.ghostTreeDepth) ~= 0
                    G.ghostTreeDepth = G.ghostTreeDepth - 1;
                end

                if G.ghostTreeDepth == 0
                    % we are finished, no more ghosts
                    ret = [];
                    return
                end

                % otherwise we are at at depth where wrapDimFlags[ghostTreeDepth] == 0
                G.wrapDimFlags(G.ghostTreeDepth) = 1;

                % calculate this (wrapped) dimension of the ghost
                dimVal = G.queryPoint(G.kdTree.wraps(G.ghostTreeDepth));
                dimClosest = 0.0;
                if G.queryPoint(G.kdTree.wraps(G.ghostTreeDepth)) < G.kdTree.wrapPoints(G.ghostTreeDepth)/2.0
                    % wrap to the right
                    dimVal = dimVal + G.kdTree.wrapPoints(G.ghostTreeDepth);
                    dimClosest = dimClosest + G.kdTree.wrapPoints(G.ghostTreeDepth);
                else
                    % wrap to the left
                    dimVal = dimVal - G.kdTree.wrapPoints(G.ghostTreeDepth);
                end
                G.currentGhost(G.kdTree.wraps(G.ghostTreeDepth)) = dimVal;
                G.closestUnwrappedPoint(G.kdTree.wraps(G.ghostTreeDepth)) = dimClosest;

                % finally, move back down the tree to the left-most possible leaf, 
                % marking the path with 0s, and populating approperiate dimension of
                % ghost point with values
                while G.ghostTreeDepth < G.kdTree.numWraps
                    G.ghostTreeDepth = G.ghostTreeDepth + 1;
                    G.wrapDimFlags(G.ghostTreeDepth) = 0;
                    G.currentGhost(G.kdTree.wraps(G.ghostTreeDepth)) = G.queryPoint(G.kdTree.wraps(G.ghostTreeDepth));
                    G.closestUnwrappedPoint(G.kdTree.wraps(G.ghostTreeDepth)) = G.currentGhost(G.kdTree.wraps(G.ghostTreeDepth));
                end

                % check if closest point in unwrapped space is further than best distance
                if G.kdTree.distanceFunction(G.closestUnwrappedPoint, G.currentGhost) > bestDist
                    continue
                end
                
                ret = G.currentGhost;
                return
            end

        end      
    end
end

