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

classdef funnelStruct < handle
    %Defines the edge data structure with the following fields
    properties
        index
        
        %funnel related attributes
        time
        trajectory
        RofA
        
        %search graph based attributes
        parent
        child
        cost
        
        withinObstacle %variable to indicate whether this edge is in collision
        
    end
    methods
        %constructor class - initialises with the parent, child and an unique id
        function obj = funnelStruct(id, funnel, cost) 
            
            obj.parent = [];
            obj.child = [];
                
            if nargin == 0 %useful for dummy edge intialisations
                obj.index = NaN;
                obj.time = NaN;
                obj.trajectory = [];
                obj.RofA = [];
                
                obj.cost = inf;
                return
            else
                obj.index = id;
                obj.time = funnel.time;
                obj.trajectory = funnel.trajectory;
                obj.RofA = funnel.RofA;
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