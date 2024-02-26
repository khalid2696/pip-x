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

%Defining a class to abstract information about funnels in the funnel-network  
classdef searchFunnel < handle
    properties
        
        funnelLibrary
        stateDimension
        resolution
        
        numNodes 
        numFunnelEdges
        
        graphNodes
        funnelEdges
        
        startNode
        goalNode
    end
    methods
        function obj = searchFunnel(library)
            
            obj.startNode = []; %will be updated in runtime
            obj.goalNode  = []; %will be updated in runtime
                     
            obj.numNodes = 0;
            obj.numFunnelEdges = 0;

            obj.graphNodes = nodeStruct();
            obj.funnelEdges = funnelStruct();
            
            obj.funnelLibrary = library;
            obj.stateDimension = 6; %no. of states
            obj.resolution = 360/length(library);
            
        end
        
        function obj = addNode(obj,node)
                obj.numNodes = obj.numNodes + 1;
                obj.graphNodes(obj.numNodes) = node;
        end
        
        function obj = addFunnelEdge(obj,funnel)
                obj.numFunnelEdges = obj.numFunnelEdges + 1;
                obj.funnelEdges(obj.numFunnelEdges) = funnel;
                if isnan(funnel.index)
                    funnel.index = obj.numFunnelEdges;
                end
        end
        
        function check = inFunnel(obj,point)
            N = obj.numFunnelEdges;
            check = 0;
            for i=1:N
                traj = obj.funnelEdges(i).trajectory;
                funnel = obj.funnelEdges(i).RofA;        
                %at this stage you have the trajectory and
                %the RofA along the knot points
                if(notInBoudingCircle(obj,traj,funnel,point))
                    continue
                end 

                funnelSize = size(traj,2);
                vanDerSequence = ceil(vdcorput(obj,funnelSize,2)*funnelSize);
                for j=1:funnelSize
                    state = traj(:,vanDerSequence(j));
                    RofA = funnel(:,:,vanDerSequence(j));
                    if(inBasin(obj,state,RofA,point))
                        check = 1;        
                        return
                    end
                end
            end
        end
        
        function funnel = steer(obj,parentNode,desiredPose)

            [t,x,P] = findFunnel(obj,parentNode.pose,desiredPose); 
            %Determine which state along the trajectory is the closest to the
            %sampled point, that would be the new node that would be added to the
            %search tree
            N = length(x);
            distances = zeros(N,1);
            for i = 1:N
                distances(i) = euclidianDist(obj,x(:,i)',desiredPose); 
            end
            [~, nodePosition] = min(distances); 
            
            time = t(nodePosition:end)-t(end) + parentNode.timeToGoal; %offsetting the time by parentNode's time
            trajectory = x(:,nodePosition:end);
            RofA = P(:,:,nodePosition:end);  %end is the parentNode pose

            funnel = struct('time',time,'trajectory',trajectory,'RofA',RofA);
        end
            
        function check = leakProof(obj,neighbor,trajectory,RofA)
            check = 1;

            neighborNewPose = trajectory(:,1); %inlet of the new funnel
            P = RofA(:,:,1);

            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy 
            E = Basis'*inv(P)*Basis; %projecting 6D space to 2D ellipse
            inlet = inv(E);
            pause('on')

            N = length(neighbor.children);
            for i=1:N
                temp = neighbor.childEdges(i); %neighbor's children
                c = obj.funnelEdges(temp).trajectory(:,end); %outlet of the child funnels
                P = obj.funnelEdges(temp).RofA(:,:,end);
                E = Basis'/P*Basis;
                outlet = inv(E);
                if(~ellipseinEllipse(obj,c,outlet,neighborNewPose,inlet))  %ellipse-in-ellipse check     
                    check=0;    
                    return
                end
            end
        end
        %--------------------------------------------------------
        %plotting functions
        %--------------------------------------------------------
        function drawSearchFunnel(obj)
            
            for i=1:obj.numNodes
                thisNode = obj.graphNodes(i);
                
                if thisNode.withinObstacle || isnan(thisNode.parent)
                    continue
                end
                thisFunnel = obj.funnelEdges(thisNode.parentFunnelEdge);
                if ~thisFunnel.withinObstacle
                    drawFunnel(obj,thisFunnel,1);
                end
            end

            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
            %plot(G.startNode.pose(1), G.startNode.pose(2), 'dm', 'MarkerSize', 6, 'LineWidth', 3.5)
        end
        
        %draws the goal branch by backtracking through the parent pointers
        function drawGoalBranch(obj)

            start = obj.startNode.pose;
            goal = obj.goalNode.pose;
            delta = 0.01;
            
            %Constructing the goal branch by backtracking through parent pointers
            tempNode = obj.startNode;
            while 1
                
                if abs(tempNode.pose - goal) < delta
                    break
                end
                
                if (isnan(tempNode.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end

                tempFunnel = obj.funnelEdges(tempNode.parentFunnelEdge);
                
                %P = tempFunnel.RofA;
                x = tempFunnel.trajectory;
                drawFunnel(obj,tempFunnel,2);
                plot(x(1,:),x(2,:),'-.c','LineWidth',2.5);
                
                tempNode = obj.graphNodes(tempNode.parent);
            end

            %plot(start(1),start(2),'sg','LineWidth',3,'MarkerSize',7);
            plot(start(1),start(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
            plot(goal(1),goal(2),'xr','LineWidth', 3,'MarkerSize',7);
           
        end
        
        %draws funnel defined by trajectory, x and ellipsoids, P along the knot points
        function drawFunnel(obj,funnel,status)
            N = length(funnel.trajectory);
            for j=N-20:-2:1 %change it to -1 to get more pretty plots
                P = funnel.RofA(:,:,j);
                xt = funnel.trajectory(:,j);
                drawEllipse(obj,xt,P,status);
            end

            if status==0
                drawDeletedTrajectory(obj,funnel);
            else
                drawTrajectory(obj,funnel);
            end
        end

        %draws trajectory, x in 2D space
        function drawTrajectory(obj,funnel)
            %figure(1)
            traj = funnel.trajectory';
            plot(traj(:,1),traj(:,2),':k','LineWidth',1.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ok','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            plot(traj(end,1),traj(end,2),'.k','LineWidth',2,'MarkerSize',4); %interior of the funnel
        end

        %draws deleted trajectory, x in white
        function drawDeletedTrajectory(obj,funnel)
            %figure(1)
            traj = funnel.trajectory';
            plot(traj(:,1),traj(:,2),':w','LineWidth',2.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ow','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            %plot(trajectory(end,1),trajectory(end,2),'.k','LineWidth',2,'MarkerSize',4); %interior of the funnel
        end

        %draws an ellipse defined by xTMx<1 with centre c
        %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        function drawEllipse(obj,center,RofA,status)

            N = 12; %50 for more pretty plots
            th = linspace(-pi,pi,N);
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
            E = Basis'/RofA*Basis;
            %ell = E^(1/2)*[cos(th); sin(th)];
            ell = sqrtm(E)*[cos(th); sin(th)];

            if status == 2 
                color = [0 0.9 0.1]; alpha = 0.5; %green
            elseif status == 1
                color = [0.8 0.8 0.8]; alpha = 0.5; %gray
            else
                color = [0.99 0.99 0.99]; alpha = 0.8; %opaque
            end

            fill(center(1) + ell(1,:),center(2) + ell(2,:),color,'edgeColor',color,'FaceAlpha',alpha);
        end

        %3D - (x,y) + time - counterparts of the former functions
        %draws the funnel with time-trajectory and ellipsoids information at the knot points
        function drawFunnelwithTime(obj,funnel)
            %figure(2)
            N = length(funnel.trajectory);
            for j=N:-1:1
                P = funnel.RofA(:,:,j);
                xt = funnel.trajectory(:,j);
                t = funnel.time(:,j);
                drawEllipsewithTime(obj,t,xt,P);
            end
            drawTrajectorywithTime(obj,funnel); 
        end

        %draws trajectory with time in a 3D plot
        function drawTrajectorywithTime(obj,funnel)
            %figure(2)
            traj = funnel.trajectory';
            plot3(traj(:,1),traj(:,2),obj.time,'--k','LineWidth',1.5);
            %Plotting the end and start points
            plot3(traj(1,1),traj(1,2),obj.time(1),'ok','LineWidth',1.5,'MarkerSize',3);
            plot3(traj(end,1),traj(end,2),obj.time(end),'.k','LineWidth',2,'MarkerSize',4);
        end

        %draws 2D ellipses "elevated" along the time axis
        %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        function drawEllipsewithTime(obj,time,center,RofA,status)
            %figure(2);
            hold on
            N = 50;
            th = linspace(-pi,pi,N);
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
            E = Basis'/RofA*Basis;
            ell = E^(1/2)*[cos(th); sin(th)];
            time = time*ones(length(ell),1);

            if status == 2
                color = [0 0.9 0.1]; 
            elseif status == 1
                color = [0.8 0.8 0.8];
            else
                color = [1 1 1];
            end

            fill3(center(1) + ell(1,:),center(2) + ell(2,:),time, color,'edgeColor',color,'FaceAlpha',0.1);
        end   

        %draws the goal branch along the time axis as well
        function drawGoalBranchwithTime(obj,searchGraph)
            %figure(2)
  
            %Constructing the goal branch by backtracking through parent pointers
            start = searchGraph.startNode.pose;
            goal = searchGraph.goalNode.pose;
            delta = 0.001;
            
            %Constructing the goal branch by backtracking through parent pointers
            temp = searchGraph.startNode;
            finishTime = temp.timeToGoal;
            
            while 1
                if abs(temp.pose - goal) < delta
                    break
                end
                
                if (isnan(temp.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end
                
                tempFunnel = obj.funnelEdges(temp.parentEdge);
                x = tempFunnel.trajectory;
                t = tempFunnel.time;
                
                drawFunnelWithTime(obj,tempFunnel,2);
                plot3(x(:,1),x(:,2),t,'-.m','LineWidth',3);
                  
                temp = searchGraph.graphNodes(temp.parent);
                
            end

            plot3(start(1),start(2),finishTime,'sg','LineWidth',3,'MarkerSize',7);
            plot3(goal(1),goal(2),0,'xr','LineWidth', 3,'MarkerSize',7);
        end
        
    end
    
    methods (Access = private)
        
        %distance function
        function dist = euclidianDist(obj,v,w)
            dist = sqrt((v(1)-w(1))^2 + (v(2)-w(2))^2);
            %dist = sqrt(sum((v - w).^2,2));
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
        
        %Extracting the funnel-edge (parent to sampled node) from the trajectory library
        %!!returns trajectory projected down to x and y, but funnel in 6D space!!
        function [t,x,P] = findFunnel(obj,parentPose,desiredPose) 

            %finding out in which sector does the sampled node lies in 
            slope = atan2((desiredPose(2)-parentPose(2)),(desiredPose(1) - parentPose(1)))*180/pi;
            slope = slope + (slope<0)*360;
            index = floor(slope/obj.resolution)+1;

            n = obj.stateDimension;
            %extracting the corresponding funnel from the library
            tempFunnel = obj.funnelLibrary(index);
            N = length(tempFunnel)-1;
            t = zeros(1,N);
            x = zeros(2,N); %just x and y
            P = zeros(n,n,N);

            for j=1:N
                t(j) = tempFunnel(j).time;
                x(:,j) = tempFunnel(j).trajectory(1:2) + parentPose'; %translating the inFunnel to the parent node's location
                P(:,:,j) = tempFunnel(j).RofA;                    %since position is a "cyclic" dimension 
            end  
        end
        
        
        
        %Function to perform a course-check whether the point is not in the
        %bounding circle of the funnel, returns 1 if point is outside the circle
        function pass = notInBoudingCircle(obj,x,RofA,point)
            pass = 0;
            initialState  = x(1:2,1);
            finalState = x(1:2,end);
            midState = (initialState+finalState)/2; %computing the approx centre of the trajectory

            radius = 1.3*euclidianDist(obj,initialState,finalState)/2;
            if(euclidianDist(obj,midState,point)>radius)
                pass = 1;
            end
        end
        
        %Function to determine whether a point lies inside an ellipse or not
        function check = inBasin(obj,x,RofA,point)
            check = 0;
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx

            E = Basis'*(RofA\Basis);   %Basis'*inv(RofA)*Basis
            %P = inv(E);
            if(point'-x)'*(E\(point'-x))<=1 % (point'-x)'*inv(E)*(point'-x)
                check=1;
                return
            end
        end
        
        %checks whether E1 lies completely within E2
        function pass = ellipseinEllipse(obj,c1,E1,c2,E2)
            
            wrapN = @(x, n) (1 + mod(x-1, n)); %function handle for circular indexing
            
            pass = 1;
            if(c1-c2)'*E2*(c1-c2)>1 %if the centre itself doesn't lie in the ellipse, return  
                pass=0;
                return
            end   
            N = size(E1,1);
            E = inv(E1);
            [~, D, V] = svd(E);

            theta = linspace(0,2*pi,9); %checking every 2*pi/8 (45 degrees)

             
            %N = length(X);
            for i = 1:N
                index1 = wrapN(i,N);
                index2 = wrapN(i+1,N);

                a = sqrt(D(index1,index1)); %access the principal axis components (magnitude)
                b = sqrt(D(index2,index2)); 
                Basis = [V(:,index1) V(:,index2)]; %accessing the corresponding principal axis vectors (direction)

                for j=2:length(theta) %0 and 2*pi are same (starting from 2)
                    checkPoint = Basis*[a*cos(theta(j)) b*sin(theta(j))]' + c1;
                    %[a*cos(theta) b*sin(theta)] are points on the ellipse at theta spacing
                    %Multiply this by the basis (eigen) vector and translate to c1 to get
                    %the coordinates in the global frame
                    if(checkPoint-c2)'*E2*(checkPoint-c2)>1
                        pass=0;
                        return
                    end
                end      
            end
        end

    end
end