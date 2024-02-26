% The MIT License (MIT)
%
% Copyright 2022 Mohamed Khalid M Jaffar, University of Maryland
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
% The software is provided "As Is", without warranty of any kind, express or
% implied, including but not limited to the warranties of merchantability,
% fitness for a particular purpose and noninfringement. In no event shall the
% authors or copyright holders be liable for any claim, damages or other
% liability, whether in an action of contract, tort or otherwise, arising from,
% out of or in connection with the software or the use or other dealings in
% the software.

classdef nodeStruct < handle
    %Defines the node data structure with the following fields
    properties
        index
        pose
        cost
        parent
        parentEdge
        children
        childEdges
        
        %for D*-lite
        inNeighbors
        inEdges
        outNeighbors
        outEdges
        
        %for faster online runtime
        inEdgeCosts
        outEdgeCosts
        
        %For collision detection
        withinObstacle %flag
        
        %For priority queue
        inHeap
        heapIndex
        
        % the following are used more for cost-based motion planning
        status     % 0 - unvisited, 1-in open list 2-in closed list
        key
        costToGoal % cost through the graph (adding up the edge costs in the solution branch)
        lmc        % local minimum cost (this is the estimate through the best neighbor)
        heuristicValue %temporarily storing the heuristic value for faster processing
        
        %funnel related attributes
        timeToGoal
        parentFunnelEdge
    end
    methods
        %constructor class - initialises with the pose and an unique id
        function obj = nodeStruct(id, state)
            
            if nargin == 0 %useful for creation of dummy nodes
                obj.index = NaN;
                obj.pose = [NaN NaN];
            else
                obj.index = id;
                obj.pose = state;
            end
            
            obj.cost = inf;
            obj.parent = NaN; %NaN if using indices %[] if using node pointers
            obj.parentEdge = NaN;
            
            obj.children = [];
            obj.childEdges = [];
            
            obj.inEdges = [];
            obj.inNeighbors = [];
            obj.outEdges = [];
            obj.outNeighbors = [];
            
            obj.withinObstacle = 0; %by default, this flag is false - no collision
            
            obj.inHeap = false;
            obj.heapIndex = -1;
            
            obj.status = 0;
            obj.key = [inf inf];
            obj.costToGoal = inf;
            obj.lmc = inf;
            obj.heuristicValue = inf;
            
            obj.timeToGoal = inf;
            obj.parentFunnelEdge = NaN;
        end 
        
    end
end

