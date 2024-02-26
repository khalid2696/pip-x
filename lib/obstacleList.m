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

%Class definition for the random forest environment with collision-checking routines
classdef obstacleList < handle
    properties
        
        envLB
        envUB
        numObstacles %useful for keeping track of num of active obstacles
        toleranceLimit
        sensorRadius %sensor radius of the robot
        
        %list of obstacles with obstacleStruct datatype
        obstacles
        obstacleTree
     
        %internal use
        indexOfLast
        
    end
    methods
        %constructor class - initialises with the position, size and an unique id
        function obj = obstacleList(envLB,envUB,epsilon,type)
            
            obj.envLB = envLB;
            obj.envUB = envUB;
            obj.numObstacles = 0;
            obj.indexOfLast = 0;

            obj.sensorRadius = 14;
            obj.toleranceLimit = epsilon; %15        
            
            if nargin<4 %type - 2 #addition/deletion of obstacles
                return
            end
            
            if type == 1 %maze with sensing
                distFunct = @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2)); %distance function
                obj.obstacleTree = KDTree(2, distFunct); %initialise the tree, 2 - num of dimensions
            end
        end
        
        %whenever some changes happen to the underlying data structure
        %it needs to be returned, hence have to include "obj = funcName()"
        
        %functions to add and remove obstacles to the list
        function obj = addObstacle(obj,obstacle)
            
            if isnan(obstacle.index)
                obstacle.index = obj.indexOfLast+1;
            end
            obj.obstacles{end+1} = obstacle;
            obj.indexOfLast = obj.indexOfLast+1;
            obj.numObstacles = obj.numObstacles+1; %increment the total #obstacles by 1
        end
        
        function obj = initialiseObstacleTree(obj)
            
            for i=1:obj.indexOfLast
                thisObstacle = obj.obstacles{i};
                thisObstacle.status = 0; %at the start, all the obstacles are unexplored
                
                %add to the obstacle kdTree
                tempKDnode = KDTreeNode(thisObstacle.location); %changing it to pose instead of position
                tempKDnode.payload = thisObstacle;
                obj.obstacleTree.kdInsert(tempKDnode);
            end
            
        end

        function exploredObstacles = senseObstacles(obj,robotLocation)
            
            tempKDTree = obj.obstacleTree;
            range = 1.15*obj.sensorRadius; %trying to make up for circleRadius
            obstaclesInRange = tempKDTree.kdFindWithinRangePayload(range, robotLocation);
            
            exploredObstacles = {};
            
            for i=1:length(obstaclesInRange)
                tempObstacle = obstaclesInRange{i};
                
                if tempObstacle.status == 1 %if it's already explored, continue
                    continue
                end
                
                exploredObstacles{end+1} = tempObstacle;
                tempObstacle.status = 1;
            end
            
        end
        
        function addedObstacles = addDynamicObstacles(obj,n,startPose,goalPose) %n - number of obstacles
            
            addedObstacles = cell(n,1);
            
            sizeRange = [2 4]; %specify the range of workspace and size everytime

            i = 1;
            while i<=n
                location = rand(1,2).*(obj.envUB - obj.envLB) + obj.envLB;
                size = sizeRange(1) + (sizeRange(2) - sizeRange(1))*rand();

                if (euclidianDist(obj,location,goalPose) < size+2) || (euclidianDist(obj,location,startPose) < size+2)
                    %disp('Another one bites the dust!')
                    continue %explicitly avoid obstacles occluding start or goal location
                end

                %initialise an obstacle and add it to the list
                randomObstacle = obstacleStruct(obj.indexOfLast+1,location,size);
                addObstacle(obj,randomObstacle);
                addedObstacles{i} = randomObstacle;
                i = i+1;
            end
 
        end
        
        function addedObstacles = addRandomObstacles(obj,n,centre,sizeRange) %n - number of obstacles
            
            addedObstacles = cell(n,1);
            offset = obj.sensorRadius/2;
            
            if(nargin == 2)
                %centre = 100*rand([1 2]);   %useful while debugging, where we don't have to
                centre = rand(n,2).*(obj.envUB - obj.envLB) + obj.envLB;
                sizeRange = [2 4]; %specify the range of workspace and size everytime
                
                for i=1:n
                    location = centre(i,:);
                    size = sizeRange(1) + (sizeRange(2) - sizeRange(1))*rand(); 
                        
                    %initialise an obstacle and add it to the list
                    randomObstacle = obstacleStruct(obj.indexOfLast+1,location,size);
                    addObstacle(obj,randomObstacle);
                    addedObstacles{i} = randomObstacle;
                end
                
                return
            end
                
            if (nargin == 3)    %implies doesn't include the size range
                sizeRange = [2 4];
            end
            
            for i=1:n
                %assign random locations and size
                r = (obj.sensorRadius-offset)*rand() + offset;
                theta  = 2*pi*rand();
                location = [centre(1)+r*cos(theta) centre(2)+r*sin(theta)];
                size = sizeRange(1) + (sizeRange(2) - sizeRange(1))*rand();

                %initialise an obstacle and add it to the list
                randomObstacle = obstacleStruct(obj.indexOfLast+1,location,size);
                addObstacle(obj,randomObstacle);
                addedObstacles{i} = randomObstacle;
            end     
        end
        
        function obj = removeObstacle(obj,F,G,obstacle)
            
            if(obj.numObstacles < 1 || obstacle.status == 0)
                return %no obstacle to remove or if obstacle has already been removed
            end
            
            obstacle.status = 0;
            obj.numObstacles = obj.numObstacles-1; %decrement the total #obstacles by 1
            
            tempNodes = obstacle.nodesWithin;
            for i=1:length(tempNodes)
                tempNodes{i}.withinObstacle = 0;
            end
            
            tempEdges = obstacle.edgesWithin;
            for i=1:length(tempEdges)
                thisEdge = tempEdges(i);
                
                head = G.graphNodes(thisEdge.parent);
                tail = G.graphNodes(thisEdge.child);
                newCost = euclidianDist(obj,head.pose,tail.pose);
                
                tempEdges(i).withinObstacle = 0;
                tempEdges(i).cost = newCost;
                
                thisFunnel = F.funnelEdges(thisEdge.index);
                thisFunnel.cost = newCost;
                thisFunnel.withinObstacle = 0;
            end      
        end
        
        function deletedObstacles = removeRandomObstacles(obj,F,G,n)
            
            deletedObstacles = cell(n,1);
            i = 0;
            while i<n
                index = randi([1 obj.indexOfLast]);
                tempObstacle = obj.obstacles{index};
                if(tempObstacle.status == 0) 
                    continue %if the obstacle is inactive continue
                end
                removeObstacle(obj,F,G,tempObstacle);
                i = i+1;
                deletedObstacles{i} = tempObstacle;
            end
            
        end
        
        function nodes = findNodesWithinEachObstacle(obj,G,tree,obstacle)
                
            if obstacle.status == 0 
                nodes = {};
                return %if the obstacle is not active continue
            end

            centre  = obstacle.location;
            epsilon = obstacle.radius;

            nodes = tree.kdFindWithinRangePayload(epsilon,centre);
            obstacle.nodesWithin = nodes;   

            for j=1:length(nodes)
                tempNode = nodes{j};
                tempNode.withinObstacle = 1; %make the within obstacle flag true
            end 
        end
            
        %functions to determine which nodes and edges are within obstacles
        function collisionNodes = findNodesWithinObstacles(obj,G,tree,obstacles)
            
            collisionNodes = [];
             for i = 1:length(obstacles)
                nodes = findNodesWithinEachObstacle(obj,G,tree,obstacles{i});   
                collisionNodes = [collisionNodes nodes'];
            end
        end
      
        
        function edges = findEdgesWithinEachObstacle(obj,F,G,tree,obstacle)
                
            if obstacle.status == 0 
                edges = [];
                return %if the obstacle is not active continue
            end

            centre  = obstacle.location;
            epsilon = sqrt(obstacle.radius^2+obj.toleranceLimit^2);

            nodes = tree.kdFindWithinRangePayload(epsilon,centre);         
            edgeIndices = [];

            for j=1:length(nodes)
                thisNode = nodes{j};

                %pre-allocating memory to edge indices
                edgesRaw = NaN((length(thisNode.inEdges)+length(thisNode.outEdges)),1);
                s = 0;
                for k=1:length(thisNode.inEdges)
                    tempEdgeIndex = thisNode.inEdges(k);
                    tempEdge = G.graphEdges(tempEdgeIndex);

                    tempHead = G.graphNodes(tempEdge.parent);
                    tempTail = G.graphNodes(tempEdge.child);

                    if obj.edgeCollisionFree(tempHead.pose,tempTail.pose)
                        continue
                    end

                    s = s+1;
                    edgesRaw(s) = tempEdgeIndex;

                end

                for k=1:length(thisNode.outEdges)
                    tempEdgeIndex = thisNode.outEdges(k);
                    tempEdge = G.graphEdges(tempEdgeIndex);

                    tempHead = G.graphNodes(tempEdge.parent);
                    tempTail = G.graphNodes(tempEdge.child);

                    if obj.edgeCollisionFree(tempHead.pose,tempTail.pose)
                        continue
                    end

                    s = s+1;
                    edgesRaw(s) = tempEdgeIndex; 
                end

                edgeIndices = [edgeIndices unique(edgesRaw(1:s))']; %to remove any possible duplicates    
            end

            if(isempty(edgeIndices)) %if no edge is in collision, continue
                edges = [];
                return
            end
            
            %edgeIdices - all the edges that are in collision with this obstacle 
            %now we extract the edge pointers and save it
            edges(length(edgeIndices),1) = edgeStruct();
            for k=1:length(edgeIndices)
                tempEdgeIndex = edgeIndices(k);
                tempEdge = G.graphEdges(tempEdgeIndex);

                tempEdge.cost = inf;
                tempEdge.withinObstacle = 1;
                
                tempFunnel = F.funnelEdges(tempEdgeIndex);
                tempFunnel.cost = inf;
                tempFunnel.withinObstacle = 1;

                edges(k) = tempEdge;
            end
            %store the edges data to this obstacle 
            obstacle.edgesWithin = edges;
        end
        
        %have to complete editing this part
        function collisionEdges = findEdgesWithinObstacles(obj,F,G,tree,obstacles)
            
            collisionEdges = [];
            %for i = 1:obj.indexOfLast
            for i = 1:length(obstacles)
                edges = findEdgesWithinEachObstacle(obj,F,G,tree,obstacles{i});
                collisionEdges = [collisionEdges edges'];
            end
        end
                
        function modifiedEdges = getModifiedEdges(obj,F,G,tree,obstacles)
            
            if(length(obstacles)<1)
                modifiedEdges = [];
                return
            end
            
            if (obstacles{1}.status == 1) %this list comprises of added obstacles
                %so determine the edges in collision first
                findNodesWithinObstacles(obj,G,tree,obstacles);
                modifiedEdges = findEdgesWithinObstacles(obj,F,G,tree,obstacles);
                return
            end
            
            modifiedEdges = [];
            for i=1:length(obstacles)
                modifiedEdges = [modifiedEdges obstacles{i}.edgesWithin'];
            end
        end
        
        %plotting functions
        
        %function to draw all obstacles
        function drawAllObstacles(obj)            
            
            for i=1:obj.indexOfLast
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0)
                    obj.drawDeletedObstacle(thisObstacle);
                else %plot inactive obstacles with dashed circles
                    obj.drawActiveObstacle(thisObstacle);
                end
            end
        end
        
        %Function to draw shaded circular obstacles
        function drawActiveObstacle(obj,obstacle)
            
            center = obstacle.location;
            radius = obstacle.radius;

            th = 0:pi/50:2*pi;
            xunit = radius * cos(th) + center(1);
            yunit = radius * sin(th) + center(2);
            
            %plot(xunit, yunit,'k','LineWidth', 1.1);
            fill(xunit,yunit,[0.3 0.3 0.3])
            plot(center(1),center(2),'xw')
        end
        
        %function to draw a deleted obstacle with dashed line
        function drawDeletedObstacle(obj,obstacle)
            
            color = [1 1 1];
            c = obstacle.location;
            r = obstacle.radius;
            
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + c(1);
            yunit = r * sin(th) + c(2);
            plot(xunit, yunit,'--k','LineWidth',1.1);
        end
        
        %Function to draw circle representing sensor radius
        function drawSensorRadius(obj,c)
            r = obj.sensorRadius;
            hold on
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + c(1);
            yunit = r * sin(th) + c(2);
            plot(c(1),c(2),'xk')
            plot(xunit, yunit,'-.k');
        end

        %Collision checking functions

        %Checking whether point lies within a circle
        
        function check = vertexCollisionFree(obj,v)

            check = 1;
            
            for i = 1:obj.indexOfLast
                
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end
                %Accessing the centre and radius from the obstacles file
                centre = thisObstacle.location;
                radius = thisObstacle.radius;

                %Checking if node v is inside the obstacle            
                if (euclidianDist(obj,centre,v) < radius) 
                    check = 0;
                    return
                end
            end
        end

        %returns 1 if edge is free of collision else return 0
        %edge - (v,w) v <-- parent w <-- child
        %Collision check of a line and circle
        function check = edgeCollisionFree(obj,v,w)
            
            check = 1;
            %v = edgeHead.pose;
            %w = edgeTail.pose;
            if(~vertexCollisionFree(obj,w) || ~vertexCollisionFree(obj,v))
                check = 0;
                return;
            end

            for i = 1:obj.indexOfLast
                
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end
                
                %Accessing the centre and radius from the obstacles file
                centre = thisObstacle.location;
                radius = thisObstacle.radius;
                

                %Checking if the edge (v,w) intersects the circle
                
                angleSubtend = ((centre(1)-v(1))*(w(1)-v(1)) + (centre(2)-v(2))*(w(2)-v(2)))/(euclidianDist(obj,v,w)^2);

                xProjection = v(1) + angleSubtend*(w(1)-v(1));
                yProjection = v(2) + angleSubtend*(w(2)-v(2));

                closestPoint = [xProjection, yProjection];

                %If the closest point lies within the edge and as well as at a
                %distance less than the radius, it implies collision
                if(liesInBetween(obj,v,w,closestPoint) && (euclidianDist(obj,closestPoint,centre) < radius))
                    check = 0;
                    return
                end
            end
        end
        
        function success = funnelCollisionFree(obj,funnel)
            success = 1;
            
            traj = funnel.trajectory;
            RofA = funnel.RofA;
            
            initialState  = traj(1:2,1);
            finalState = traj(1:2,end);
            midState = (initialState+finalState)/2; %computing the approx centre of the trajectory
            funnelRadius = 1.5*euclidianDist(obj,initialState,finalState)/2;
            
            %Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %E = Basis'/RofA(:,:,1)*Basis;
            %[~, D, ~] = svd(E);
            %a = sqrt(D(1,1)); %compute largest semi-major axis  value along the trajectory
                              %will be at the start of the trajectory!   
            %funnelRadius = euclidianDist(obj,initialState,finalState)/2 + a; %computing the approx radius of bounding circle
            
            
            for i = 1:obj.indexOfLast
               
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end

                if(boundingCircleCheck(obj,midState,funnelRadius,thisObstacle)) %if the funnel is sufficiently far off
                    continue                                         %from the obstacle return with 1   
                end

                if(~funnelCircleCollision(obj,funnel,thisObstacle))
                    success=0;
                    return
                end
            end
        end
    end
    

    %if we have to declare functions that are required to be 
    %private in scope. Can be accessed from only within this class
    methods (Access = private)
        %distance function
        function dist = euclidianDist(obj,v,w)
            dist = sqrt(sum((v - w).^2,2));
        end
        
        %if a point lies within a linesegment VW returns 1
        function check = liesInBetween(obj,v,w,P)
            check = 1;
            if (P - v)*(P - w)' > 0
                check = 0;
            end
        end
        
        %rough check by drawing a bounding circle on the ellipse
        function success = firstPass(obj,c,r,a)
            if norm(c)>(a+r)
                success=1;
            else
                success=0;
            end
        end

        %Function to generate Van Der Corput sequence %N - array length, b - base
        function s = vdcorput(obj,N,b)  %output - N+1 (starting with ZERO)
            s = zeros(N,1);
            for i = 1:N
                a = basexpflip(obj,i,b);
                g = b.^(1:length(a));
                s(i) = sum(a./g);
            end    
        end

        %Reversed base-b expansion of positive integer k
        function a = basexpflip(obj,k,b) 
            j = fix(log(k)/log(b)) + 1;
            a = zeros(1,j);
            q = b^(j-1);
            for i = 1:j
               a(i) = floor(k/q);
               k = k - q*a(i);
               q = q/b;
            end
            a = fliplr(a);
        end
        
        %Function to perform a course-check whether the point is not in the
        %bounding circle of the funnel, returns 1 if point is outside the circle
        function pass = boundingCircleCheck(obj,midState,funnelRadius,obstacle)
            pass = 0;
            center = obstacle.location;
            radius = obstacle.radius; 
            if(euclidianDist(obj,midState,center)>funnelRadius+radius)
                pass = 1;
                return
            end
        end
        
        %checks collision b/w funnel and each circular obstacle
        function success = funnelCircleCollision(obj,funnel,obstacle)
            
            RofA = funnel.RofA;
            success = 1;    
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            funnelSize = size(RofA,3);
            vanDerSequence = ceil(vdcorput(obj,funnelSize,2)*funnelSize);

            for i = 1:funnelSize
                index = vanDerSequence(i);
                E = Basis'/RofA(:,:,index)*Basis;
                %RofA = inv(E); %RofA is technically inverse(E), but I'm avoiding
                %taking double inverse in ellipse collision checking sub-routine 
                x = funnel.trajectory(:,index);

                if(~ellipseCircleCollisionFree(obj,x,E,obstacle))
                    success = 0;
                    return
                end
            end
        end

        %Collision-check between an ellipse and list of circular obtacles
        function success = ellipseCircleCollisionFree(obj,ellipseCentre,M,obstacle)

            success = 1;
            %for algebraic analaysis
            [~, D, V] = svd(M);

            a = sqrt(max(diag(D))); b = sqrt(min(diag(D)));
            c = sqrt(a^2 - b^2); %Focal length

            pointA = [c 0]'; pointB = [-c 0]';

            %Transforming along principal coordinates
            circleCentre = obstacle.location';
            circleRadius = obstacle.radius;
            pointC = V*(circleCentre - ellipseCentre);

            if firstPass(obj,pointC,circleRadius,a)
                return
            end

            %Detecting the point P of intersection should come over here
            pointP = pointOfIntersection(obj,pointA, pointC, circleRadius);
            dist1 = norm(pointP - pointA) + norm(pointP - pointB);

            pointP = pointOfIntersection(obj,pointB, pointC, circleRadius);
            dist2 = norm(pointP - pointA) + norm(pointP - pointB);

            if(dist1 < 2*a || dist2 < 2*a)
                success = 0;
                return
            end
        end

        %Function to determine the point of intersection b/w
        %a ray from a point A to the centre and the circle
        function P = pointOfIntersection(obj,point,center,radius)
            phi = atan2(point(2)-center(2), point(1)-center(1));
            x = center(1) + radius * cos(phi);
            y = center(2) + radius * sin(phi);
            P = [x;y];
        end
    end
end