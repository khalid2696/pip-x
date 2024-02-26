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

%Defining a class to store the graph data structure representing the funnel-network
classdef searchGraph < handle
    properties
        
        numNodes 
        numEdges
        
        graphNodes
        graphEdges
        
        startNode
        goalNode
        
    end
    methods
        function obj = searchGraph(nodes,edges)
            
            if(nargin == 0)
            
                obj.numNodes = 0;
                obj.numEdges = 0;
                
                obj.graphNodes = nodeStruct();
                obj.graphEdges = edgeStruct();
                
                obj.startNode = []; %will be updated in runtime
                obj.goalNode  = []; %will be updated in runtime
                return
            end
            
            obj.graphNodes = nodes;
            obj.graphEdges = edges;
            
            obj.numNodes = length(nodes);
            obj.numEdges = length(edges);
            
            obj.startNode = []; %will be updated in runtime
            obj.goalNode  = []; %will be updated in runtime
        end
        
        function obj = addNode(obj,node)
                obj.numNodes = obj.numNodes + 1;
                obj.graphNodes(obj.numNodes) = node;
        end
        
        function obj = addEdge(obj,edge)
                obj.numEdges = obj.numEdges + 1;
                obj.graphEdges(obj.numEdges) = edge;
                if isnan(edge.index)
                    edge.index = obj.numEdges;
                end
        end
        
        function drawSearchGraph(obj)
            %figure(2)
            nodes = NaN(obj.numNodes,2);
            for i=1:obj.numNodes
                nodes(i,:) = obj.graphNodes(i).pose;
            end
            
            % a bit of data processing for faster plotting
            edges = NaN(3*obj.numEdges,2);
            for i=3:3:3*obj.numEdges
                tempEdge = obj.graphEdges(i/3);
                edges(i-2,:) = obj.graphNodes(tempEdge.child).pose; % edge starts
                edges(i-1,:) = obj.graphNodes(tempEdge.parent).pose; % edge ends
            end 
            
            plot(edges(:,1),edges(:,2),'Color',[0.3010, 0.7450, 0.9330],'LineWidth',1.5); % [0, 0.4470, 0.7410]
            plot(nodes(:,1),nodes(:,2), 'o','Color',[0, 0.4470, 0.7410]); %[0, 0.4470, 0.7410]
            
            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)  
        end
        
        function drawSearchTree(obj)
            edges = NaN(3*obj.numNodes,2);
            for i=6:3:3*obj.numNodes
                tempNode = obj.graphNodes(i/3); %starting from 2, because 1 is goal node which doesn't have a parent 
                if(isnan(tempNode.parent))
                    continue
                end
                edges(i-2,:) = obj.graphNodes(tempNode.parent).pose; % edge starts
                edges(i-1,:) = tempNode.pose; % edge ends
            end 

            %plot(edges(:,1),edges(:,2),'Color','y','LineWidth',2.7); %[0, 0.4470, 0.7410], [0.3010, 0.7450, 0.9330] 
            plot(edges(:,1),edges(:,2),'Color',[0, 0.4470, 0.7410],'LineWidth',2.5); %1.9 %[0.3010, 0.7450, 0.9330] 
              
            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)  
        end
        
        function drawPathToGoal(obj)
    
            %drawSearchTree(obj);
            start = obj.startNode.pose;
            goal = obj.goalNode.pose;
            delta = 0.1;

            j = 1;
            %Constructing the goal branch by backtracking through parent pointers
            temp = obj.startNode;
            while 1

                if (isnan(temp.parent))
                %if (isempty(temp) || isnan(temp.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end

                goalBranch(j,:) = temp.pose;  
                temp = obj.graphNodes(temp.parent);
                if abs(temp.pose - goal) < delta
                    goalBranch(j+1,:)=goal;
                    break
                end
                j = j+1;
            end

            plot(start(1),start(2), 'sg','MarkerSize', 6,'LineWidth',2);
            plot(goalBranch(:,1),goalBranch(:,2),'-.g','LineWidth', 2.5); %2.5
            plot(goal(1),goal(2),'xr','MarkerSize', 6,'LineWidth',2);
        end
    end
end