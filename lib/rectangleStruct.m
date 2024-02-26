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

classdef rectangleStruct < handle
    %Defines a data structure for rectanglular obstacles (maze walls)
    properties
        %identifier
        index
        
        %properties of the RECTANGULAR obstacle
        location
        length
        breadth
        orientation %0 or 90 deg (horizontal or vertical)
        
        %splitting the rectangle into smaller squares for collision
        %checking
        lowerKnots
        upperKnots
        
        %nodes and the edges that are within this graph 
        %will be updated during runtime
        %nodesWithin
        %edgesWithin
        
        %later use - dynamic obstacles
        status %variable to keep track whether an obstacle is active or not
        timeIn
        timeOut
        
    end
    methods
        %constructor class - initialises with the position, size and an unique id
        function obj = rectangleStruct(id, location, size, theta)
            
            obj.index = id;
            obj.location = location;
            obj.status = 0; %by default if an obstacle is initialised
                            %we assume its unsensed yet
            
            %by default the obstacles come in at t=0 and leave at t=inf               
            obj.timeIn = 0;
            obj.timeOut = inf;
            
            %if the size of the obstacle isn't specified assign a 
            %random radius between 2 and 6 units
            if nargin == 2
                obj.length = 10 + 20*rand();
                obj.breadth = 2 + 2*rand();
                obj.orientation = randi([0 1])*90;
            elseif nargin == 4
                obj.length = size(1); %lies within range of 2 to 6 units
                obj.breadth = size(2);
                obj.orientation = theta;
            end 
        end
        
        function obj = splitRectangle(obj)
            
            c = obj.location;
            l = obj.length;
            b= obj.breadth;
            R = rotation(obj,obj.orientation);
            
            w = min(l,b);
            temp = -l/2:w:l/2; temp(end+1) = l/2;
    
            temp1(:,1) = temp; temp1(:,2) = -b/2;
            temp2(:,1) = temp; temp2(:,2) = b/2;
    
            obj.lowerKnots = c+temp1*R;
            obj.upperKnots = c+temp2*R;
        end
        
        %plotting functions
        %Function to draw sensed rectangle
        function drawSensedRectangle(obj)
            c = obj.location;
            l = obj.length;
            b = obj.breadth;
            R = rotation(obj,obj.orientation);
            v = [-l/2 -b/2; 
                  l/2 -b/2;
                  l/2  b/2;
                 -l/2  b/2;]; %vertex co-ordinates in local obstacle frame 

            vertices = c + v*R;
            fill(vertices(:,1),vertices(:,2),[0.3 0.3 0.3],'FaceAlpha',1)
        end

        function drawUnSensedRectangle(obj)
            c = obj.location;
            l = obj.length;
            b = obj.breadth;
            R = rotation(obj,obj.orientation);
            v = [-l/2 -b/2; 
                  l/2 -b/2;
                  l/2  b/2;
                 -l/2  b/2;]; %vertex co-ordinates in local obstacle frame 

            vertices = c + v*R;
            fill(vertices(:,1),vertices(:,2),[0.5 0.5 0.5],'LineStyle','--','FaceAlpha',0.01)
            %vertices(end+1,:) = vertices(1,:);
            %plot(vertices(:,1),vertices(:,2),'--k')
        end
        
        %function to draw a deleted obstacle with dashed line
        function drawDeletedRectangle(obj)
            
            color = [1 1 1];
            c = obj.location;
            l = obj.length;
            b = obj.breadth;
            R = rotation(obj,obj.orientation);
            v = [-l/2 -b/2; 
                  l/2 -b/2;
                  l/2  b/2;
                 -l/2  b/2;]; %vertex co-ordinates in local obstacle frame 

            vertices = c + v*R;
            fill(vertices(:,1),vertices(:,2),color,'FaceAlpha',0.95,'edgeColor',color)
            %fill(vertices(:,1),vertices(:,2),color,'FaceAlpha',0.05,'edgeColor',color)           
        end
        
    end
    
    methods (Access = private)
        
        function R = rotation(obj,theta)
            R = [cosd(theta) -sind(theta); 
                 sind(theta) cosd(theta)];
        end
    end
    
end