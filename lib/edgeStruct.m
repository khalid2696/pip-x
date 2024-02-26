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

classdef edgeStruct < handle
    %Defines the edge data structure with the following fields
    properties
        index
        parent
        child
        cost
        
        withinObstacle %variable to indicate whether this edge is in collision
        
    end
    methods
        %constructor class - initialises with the parent, child and an unique id
        function obj = edgeStruct(id, edge, cost) 
   
            if nargin == 0 %useful for dummy edge intialisations
                obj.index = NaN;
                obj.parent = [];
                obj.child = [];
                obj.cost = inf;
            else
                obj.index = id;
                obj.parent = edge(1);
                obj.child = edge(2);
            end
            
            if nargin < 3
                obj.cost = inf;
            else
                obj.cost = cost;
            end
            
            obj.withinObstacle = 0;
        end   
    end
end