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

classdef obstacleStruct < handle
    %Defines a data structure for circular obstacles (random forest environment)
    properties
        %identifier
        index
        
        %properties of the CIRCULAR obstacle
        location
        radius
        
        %associated rectangular obstacle
        rectangleWithin
        
        %nodes and the edges that are within this graph 
        %will be updated during runtime
        nodesWithin
        edgesWithin
        
        %later use - dynamic obstacles
        status %variable to keep track whether an obstacle is active or not
        timeIn
        timeOut
        
    end
    methods
        %constructor class - initialises with the position, size and an unique id
        function obj = obstacleStruct(id, location, size)
            
            obj.index = id;
            obj.location = location;
            obj.status = 1; %by default if an obstacle is initialised
                            %we assume its status is inactive
            
            %by default the obstacles come in at t=0 and leave at t=inf               
            obj.timeIn = 0;
            obj.timeOut = inf;
            
            %if the size of the obstacle isn't specified assign a 
            %random radius between 2 and 6 units
            if nargin == 2
                obj.radius = 2 + 4*rand();
            elseif nargin == 3
                obj.radius = size; %lies within range of 2 to 6 units
            end 
        end 
    end
end