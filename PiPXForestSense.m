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

clc
clearvars
close all

%adding paths to code libraries and funnel-library
addpath('./lib/');
addpath('./funnelLibrary/');

%configurable flags
drawFlag = 1;
saveFlag = 0;
fileCount = 1; %for saving files in /temp/ folder
%dir = ['./temp/trial' num2str(1) '/'];
%mkdir(dir);

%Assigning values to algorithm parameters
epsilon = 5;          %extend-distance
iterationLimit = 400; %Maximum number of iterations
%parameters for the shrinking r-Ball
lambda = 1e-3; %0.005 %1e-4
r0 = epsilon*1.2;

envLB = 0;
envUB = 50;
setupPlot(envLB, envUB)

O = obstacleList(envLB,envUB,epsilon,1); %obstacle class  epsilon - tolerance
                                         %mode - 1 for sensing, 2 for
                                         %dynamic addition/deletion

distFunct = @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2)); %distance function (for kDTree)
T = KDTree(2, distFunct); %initialise the tree, 2 - num of dimensions

load('FunnelLibraryDense.mat');
F = searchFunnel(library);
G = searchGraph();  %instantiate an empty graph class

waitfor(msgbox('Click on the start and goal positions respectively'));
[problemx,problemy] = ginput(2);

startPose = [problemx(1) problemy(1)];
goalPose = [problemx(2) problemy(2)];

%Plotting start and goal positions
plot(startPose(1),startPose(2), 'og','MarkerFaceColor', 'y')
plot(goalPose(1),goalPose(2), 'or','MarkerFaceColor', 'r')

%initially adding obstacles
numTrees = 25;
O.addDynamicObstacles(numTrees,startPose,goalPose); %argin - #obstacles, robot pose, goal pose, 
                                                      
O.initialiseObstacleTree();
O.senseObstacles(startPose);

%progress variables
iteration =1;             %keeps track of number of nodes in tree
startFound=0;             %Start check
status = 0;
robotMove = 0;
idleTime = 0;

traversedPathLength = 0;
remainingPathLength = inf;

goalNode = nodeStruct(1,goalPose);
goalNode.cost = 0;
G.addNode(goalNode); F.addNode(goalNode);

G.goalNode = goalNode; F.goalNode = goalNode;
G.startNode = nodeStruct(NaN,startPose); %start node will be added later 
F.startNode = nodeStruct(NaN,startPose); %start node will be updated later

T.kdInsertAsPayload(goalNode);

Q = heap(iterationLimit);

%Sample class-constructor usage

%[nearestNode, distToClosestNode] = T.kdFindNearestPayload(queryPoint)
%neighborsInRange = T.kdFindWithinRangePayload(range, queryPoint)

%nodeStruct(id, [xPose yPose])
%edgeStruct(id, [parent child], cost) 
%searchGraph(nodes,edges)
%obstacleList(obstacles)

%save the initial environment
if saveFlag
    fileCount = saveData(F,G,O,dir,fileCount);
end

%initialisation
%all nodes have infinite g and lmc value by default (constructor definition)
km = initialiseGraphSearch(G,Q);
previousPose = G.startNode;

if drawFlag
    O.drawAllObstacles();
    O.drawSensorRadius(G.startNode.pose);
end

drawnow;

if(~O.vertexCollisionFree(goalPose))
    disp('Goal inside the obstacles. No path exists!')
    drawnow
    return
end

tic

%PiP-X algorithm: Online motion planning/replanning using Funnels
while idleTime<5 && ~startCheck(G.startNode.pose,G.goalNode.pose) && iteration<iterationLimit
     
    %sampling a point at random
    sampledPoint = sampleNode(G,O,startFound,robotMove);  
    
    newNodePose = expandTree(G,T,sampledPoint,epsilon);
    
    if(~O.vertexCollisionFree(newNodePose) || F.inFunnel(newNodePose))
        continue
    end
    
    %Determine the radius of the r ball
    r = rBall(r0,lambda,iteration);
    
    %Find the best parent within an r-Ball
    [neighbors, flag] = findNeighborsInRBall(T,newNodePose,r);
    if flag %if no feasible parent found continue with the next sampling
        continue
    end
    
    thisNode = nodeStruct(iteration+1,newNodePose);
    
    %update the funnel-network with the new sampled node
    flag = constructFunnelNetwork(F,G,O,thisNode,neighbors);
    if flag
        continue
    end
    
    %adding to the data structures
    iteration = iteration+1;
    G.addNode(thisNode); F.addNode(thisNode);
    T.kdInsertAsPayload(thisNode);
     
    %Goal checking for the first time
    if(~startFound && F.inFunnel(startPose))
        %temp = G.graphNodes(iteration);
        
        startNode = nodeStruct(iteration+1,startPose);       
        
        flag = constructFunnelNetwork(F,G,O,startNode,neighbors);
        if flag
            continue
        end
        
        iteration = iteration+1;
        G.addNode(startNode); F.addNode(startNode);
        G.startNode = startNode; F.startNode = startNode;
        T.kdInsertAsPayload(startNode);

        updateVertex(G,Q,startNode,km);
        
        %compute the shortest funnel-path
        computeShortestPath(G,Q,km);
        
        startFound=1;
        fprintf('\nStart found after %d iterations! \nCurrent cost to goal is <strong>%0.2f</strong>  ',startNode.index,G.startNode.cost); 
        if drawFlag
            G.drawSearchGraph();
            F.drawSearchFunnel();
            G.drawSearchTree(); 
            F.drawGoalBranch(); O.drawAllObstacles();
            drawnow
        end
        
        if saveFlag
            fileCount = saveData(F,G,O,dir,fileCount);
        end
        
        toc
    end
    
    updateVertex(G,Q,thisNode,km);
    %compute the shortest funnel-path
    computeShortestPath(G,Q,km);
    
    for j=1:length(neighbors)
        thisNeighbor = neighbors{j};
        updateVertex(G,Q,thisNeighbor,km);
    end
    
    %km needs to be updated only if the robot moves
    km = km + computeHeuristic(previousPose,G.startNode);
    previousPose = G.startNode;
    
    %Robot movement
    if(iteration>0.6*iterationLimit && mod(iteration,5) == 0 && startFound) %0.5 and 10
              
        if(status && drawFlag)
            %G.drawSearchGraph();
            %F.drawSearchFunnel();
            G.drawSearchTree(); 
            F.drawGoalBranch(); O.drawAllObstacles();
            drawnow
        end
        
        temp = G.graphNodes(previousPose.parent);
        %for movement=1:1 %skipping along n edges (simulates robot speed)
        %    if startCheck(temp.pose,G.goalNode.pose)
        %        break
        %    end 
        %    temp = G.graphNodes(temp.parent);    
        %end
        G.startNode = temp; F.startNode = temp;
        
        if(isempty(G.startNode) || isinf(G.startNode.cost))
            G.startNode = previousPose; F.startNode = previousPose;
            idleTime = idleTime + 1;
            robotMove = 0;
            fprintf('\nNo path exists currently -- Staying at the same position!');
            fprintf('\nRobot idle for %d iterations\n',idleTime);
        else
            traversedPathLength = traversedPathLength + (previousPose.cost - G.startNode.cost);
            remainingPathLength = G.startNode.cost;
            idleTime = 0;
            robotMove = 1;
        end
        
        if(status)
            fprintf('\n\nRobot moving.... ');
            fprintf('\nTraversed distance/Remaining distance to goal - <strong>%0.2f/%0.2f</strong>',traversedPathLength,remainingPathLength);            
        end  
           
          
        km = km + computeHeuristic(previousPose,G.startNode);
        previousPose = G.startNode;  
        
        %Robot sensing
        if(mod(iteration,5) == 0)
            %modify the edge costs and add them to the priority queue
            modifyEdgeCosts(F,G,Q,O,T,km);
        end
        
        if(saveFlag && robotMove)
            fileCount = saveData(F,G,O,dir,fileCount);
        end
    end   
        
    %compute the shortest funnel-path
    status = computeShortestPath(G,Q,km);
    
   %draw the entire search tree and goal branch once every 100 iterations
    if(mod(iteration,100) == 0 && drawFlag) 
        
        fprintf('<strong>\n\nHold on.. \nRe-drawing the funnel-network and shortest funnel-paths...\n</strong>');
        G.drawSearchGraph();
        F.drawSearchFunnel();
        G.drawSearchTree(); O.drawAllObstacles();
        if(startFound && status)
            F.drawGoalBranch();
        end
        drawnow  
    end
    
    if (mod(iteration,50) == 0 && ~robotMove && startFound)
        fprintf('\nCurrent cost to goal is <strong>%0.2f</strong>  ',G.startNode.cost);
        toc
    end
    
    if(saveFlag && ~robotMove && mod(iteration,5) == 0)
        fileCount = saveData(F,G,O,dir,fileCount);
    end
    
    if startCheck(G.startNode.pose,G.goalNode.pose)
        plot(G.goalNode.pose(1),G.goalNode.pose(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
        drawnow
        break
    end
    
    if(toc>120 && ~startFound) %potentially no path exists during pre-planning phase (5 mins)
        fprintf('\nNo path exists.. Exiting in pre-planning phase itself')
        break
    end
end

%% post-processing
clearvars -except G T O F Q traversedPathLength drawFlag saveFlag fileCount startPose goalPose dir
if startCheck(G.startNode.pose,G.goalNode.pose)
    fprintf('\n\nSuccess!! The robot has reached the goal location!\n');
    success = 1;
else
    fprintf('\n\nAlgorithm Failure!!\n');
    success = 0;
end

if drawFlag
    F.drawSearchFunnel();
    %G.drawSearchTree();
    O.drawAllObstacles();
    
    setupPlot(O.envLB, O.envUB)
    G.drawSearchGraph();
    F.drawSearchFunnel();
    G.drawSearchTree();
    title('Funnel-network (graph) and shortest path funnel-tree (rooted at goal)')
    set(gca,'FontName','Helvetica','FontSize',11.5, 'FontWeight','bold');
    %O.drawAllObstacles();
end

if saveFlag
    %2 additional frames for more 'aesthetic' video
    fileCount = saveData(F,G,M,0,dir,fileCount);
    fileCount = saveData(F,G,M,0,dir,fileCount);
    save([dir 'problem.mat'],'startPose','goalPose','traversedPathLength','success','fileCount');
end

toc
%%
%-------------------------------------------------------------------------%
%end of main code
%Function definitions

%-------------------------------------------------------------------------%
%Defining distance and cost functions

%Computing Euclidian distance
function d = euclidianDist(v,u)
    d = sqrt((v(1)-u(1))^2 + (v(2)-u(2))^2);
end

%computing Geodesic distance
function d = geodesicDist(v,u)
    d = sqrt((v(1)-u(1))^2 + (v(2)-u(2))^2);
end

%Goal checking
function success = startCheck(w,start)
    success = (euclidianDist(w,start(1:2))<0.5); %tolerance limit is 5
end

%cost function definition
function cost = computeCostFn(v,u)
    cost = euclidianDist(v,u);
end
%-------------------------------------------------------------------------%
%Functions for sampling-based routines

%function to sample points
function sample = sampleNode(G,O,startFound,robotMove)
    
    start = G.startNode.pose;
    if(robotMove)
        prob = rand();
        if prob < 0.8 %0.9
            r = O.sensorRadius*rand() + 2; %epsilon away
        else
            %r = 2*O.sensorRadius*rand() + 2; %1.5
            xSample = (O.envUB - O.envLB)*rand() + O.envLB;
            ySample = (O.envUB - O.envLB)*rand() + O.envLB;
            sample = [xSample ySample];
            return
        end
        
        theta  = 2*pi*rand();
        sample = [start(1)+r*cos(theta) start(2)+r*sin(theta)];
        %plot(sample(1),sample(2),'xb');
        return
    end
    
    if(~startFound)
        bias = 0.95; %0.95
    else
        bias = 1;
    end
    prob = rand();
    if prob < bias
        %random sampling of nodes
        xSample = (O.envUB - O.envLB)*rand() + O.envLB;
        ySample = (O.envUB - O.envLB)*rand() + O.envLB;
        sample = [xSample ySample];
    else
        sample = start;
    end
    
end

%function to determine the nearest neighbor in the search tree
function newNodePose = expandTree(G,T,sampledPoint,epsilon)
    
    nearestNode = T.kdFindNearestPayload(sampledPoint);
      
    distance = euclidianDist(nearestNode.pose,sampledPoint);
    if(distance < epsilon)
        newNodePose = sampledPoint;
    else
        t = epsilon/distance;
        newNodePose(1) = (1-t)*nearestNode.pose(1) + t*sampledPoint(1);
        newNodePose(2) = (1-t)*nearestNode.pose(2) + t*sampledPoint(2);
    end
end


%Shrinking r-Ball
function r = rBall(r0,lambda,iteration)
    %Below are two versions of shrinking ball radius
    
    %1. Shrinking rate from RRT* paper
    epsilon = 5.5; r0 = 800; d = 2; iteration = iteration+1;
    r = min(r0*(log(iteration)/(iteration))^1/d,epsilon);
    
    %2. This version can also be used -- has found to have practical success 
    %(as evidenced by the results from the IJRR paper)
    %r = r0*exp(-1*lambda*iteration);
end

%Determine the parent node
function [neighbors, flag] = findNeighborsInRBall(T,newNodePose,rBall)
    
    flag = 0;
    %neighbors = nodeStruct();
    
    neighbors = T.kdFindWithinRangePayload(rBall,newNodePose);
    if isempty(neighbors) %if no parent can be found within the radius ball continue
        neighbors = nodeStruct();
        flag = 1;
        return
    end
end

function flag = constructFunnelNetwork(F,G,O,newNode,neighbors)

    N = length(neighbors);
    flag = 0;
    
    if (N<1) %if no neighbor return
        flag = 1;
        return
    end  
    
    prevEdgeCount = F.numFunnelEdges;
    for i=1:N
        thisNeighbor = neighbors{i};
        
        if thisNeighbor.withinObstacle
            continue
        end 
        
        %outFunnel for neighbor/ inFunnel for newNode 
        newFunnel = F.steer(thisNeighbor,newNode.pose);
        
        if(~O.funnelCollisionFree(newFunnel) || ~O.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
            continue
        end           
        
        edgeCost = computeCostFn(thisNeighbor.pose,newNode.pose);
        
        %Add the edge to the search tree
        funnelEdge = funnelStruct(nan,newFunnel);
        
        %This implementation of code makes a *simplifying assumption* that
        %all funnels from/to the new node are compossible with the funnels
        %at its neighbor nodes.
        %
        %This assumption is largely justified for this specific case of 2D
        %quadrotor planning because of following three reasons:
        %1. The outlet sizes are considerably small compared to the inlet sizes
        %2. The funnel library is 'dense' enough to cover an epsilon-ball
        %3. The steering function makes sure we 'translate' and 'truncate'
        %   the funnel-edges appropriately
        
        %In future, a general implementation of compossibility-check will
        %make this assumption obsolete, and will eventually modify the below
        %code-snippets
        
        %funnels and edges direction are swapped (because of reverse search)
        funnelEdge.child = thisNeighbor.index; funnelEdge.parent = newNode.index;
        funnelEdge.cost = edgeCost;
    
        F.addFunnelEdge(funnelEdge);
    
        %adding in and out Neighbors
        thisNeighbor.outNeighbors(end+1) = newNode.index;
        newNode.inNeighbors(end+1) = thisNeighbor.index;     
        
        %adding out and in edges
        tempEdge = edgeStruct(nan, [thisNeighbor.index newNode.index], edgeCost);
        G.addEdge(tempEdge);
        thisNeighbor.outEdges(end+1) = tempEdge.index;
        newNode.inEdges(end+1) = tempEdge.index;
        
        %outFunnel for newNode/ inFunnel for neighborNode 
        newFunnel = F.steer(newNode,thisNeighbor.pose);
        
        if(~O.funnelCollisionFree(newFunnel) || ~O.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
            continue
        end
        
        edgeCost = computeCostFn(newNode.pose,thisNeighbor.pose);
        
        %Add the edge to the search tree
        funnelEdge = funnelStruct(nan,newFunnel);
        
        %funnels and edges direction are swapped (because of reverse search)
        funnelEdge.child = newNode.index; funnelEdge.parent = thisNeighbor.index;
        funnelEdge.cost = edgeCost;
    
        F.addFunnelEdge(funnelEdge);
        
        thisNeighbor.inNeighbors(end+1) = newNode.index;
        newNode.outNeighbors(end+1) = thisNeighbor.index; 
        
        tempEdge = edgeStruct(nan, [newNode.index thisNeighbor.index], edgeCost);
        G.addEdge(tempEdge);
        thisNeighbor.inEdges(end+1) = tempEdge.index;
        newNode.outEdges(end+1) = tempEdge.index;
    end
    
    if prevEdgeCount == F.numFunnelEdges %i.e. if no new edge was added
        flag = 1;
    end
end

%-------------------------------------------------------------------------%
%Incremental graph-search related routines

%compute the key value of the vertex
function key = computeKey(G,node,km)
    k1 = min(node.cost, node.lmc) + computeHeuristic(node,G.startNode) + km;
    k2 = min(node.cost, node.lmc);
    key = [k1 k2];
end

%compute the heuristic from current vertex to .. 
%goal (forward search)/start (reverse search)
function h = computeHeuristic(vertex,start)
    h = 1.0*euclidianDist(vertex.pose,start.pose); 
end

% %returns TRUE if key1 is smaller than key2
% %note that it follows a lexicographic-type comparison
%The compareKey function has been defined in the heap class
function status = compareKey(key1, key2)
    status = false;
    delta = 0; %should be ideally zero (tune it later)
    if key1(1) < key2(1) + delta
        status = true;
    elseif key1(1) == key2(1)
        if key1(2) < key2(2) + delta
            status = true;
        end
    end
end

% %returns TRUE if key1 is smaller than key2
% Checking a bit further downstream of robot location
function status = compareStartKey(key1, key2)
    status = false;
    delta = 6; %should be approx same as the max edge cost (6)
    if key1(1) < key2(1) + delta
        status = true;
    elseif key1(1) == key2(1)
        if key1(2) < key2(2) + delta
            status = true;
        end
    end
end

%finds the best lmc among the neighbors of the given node
%lmc by default means the minimum one-step lookahead cost
%lmc - locally minimum cost to come
function minLMC = computeLMC(G,node)

    minLMC = inf;

    for i=1:length(node.inNeighbors)
        tempNeighbor = G.graphNodes(node.inNeighbors(i));
        
        if (tempNeighbor.parent == node.index) %explicitly avoiding loops
            continue
        end
            
        tempEdge = G.graphEdges(node.inEdges(i));
        neighborCostToGoal = min(tempNeighbor.cost,tempNeighbor.lmc);
        
        if(neighborCostToGoal + tempEdge.cost < minLMC)
            minLMC = tempNeighbor.cost + tempEdge.cost;
            node.parent = tempNeighbor.index;
            node.parentFunnelEdge = tempEdge.index; %updating the parent funnelEdge
        end
    end
end


%updates the key values of any vertices and pushes it into the queue
function updateVertex(G,Q,node,km)
    
    delta = 0.1; %0.5
    if node.pose ~= G.goalNode.pose
        node.lmc = computeLMC(G,node);
    end   
    
    if node.inHeap
        Q.remove(node);
    end
    
    %remove and push only inconsistent nodes
    %if node.cost ~= node.lmc %if inconsistent
    if abs(node.cost-node.lmc)>delta %only if substantial change
        node.key = computeKey(G,node,km);
        Q.push(node, node.key);
    end
end

%updates all the child vertices of a given node
function updateSuccessorVertices(G,Q,node,km)
    
    for i = 1:length(node.outNeighbors)
        tempChild = G.graphNodes(node.outNeighbors(i));
        updateVertex(G,Q,tempChild,km);
    end
end

%initialises the graph-search algorithm
%all nodes have infinite g and lmc value by default (constructor definition)
function km = initialiseGraphSearch(G,Q)
    
    km = 0;
    G.goalNode.lmc = 0;
    G.goalNode.key = computeKey(G,G.goalNode,km);
    Q.push(G.goalNode,G.goalNode.key);
end

%Computes (and recomputes) optimal funnel-paths 
%Makes repairs (new sample configurations) and rewires (changed obstacle-space)
function status = computeShortestPath(G,Q,km)
    
    G.startNode.key = computeKey(G,G.startNode,km);
    while G.startNode.lmc ~= G.startNode.cost || ... %Q.indexOfLast > 0
          compareStartKey(Q.topKey(),G.startNode.key)      
        
        kOld = Q.topKey();
        currentNode = Q.pop();
        
        if(isempty(currentNode)) %if the priority queue is empty, return
            break
        end
        
        currentNode.key = computeKey(G,currentNode,km);
        
        if compareKey(kOld,currentNode.key)
            Q.push(currentNode,currentNode.key);
        elseif currentNode.cost > currentNode.lmc %overconsistent, shortest path exists
            currentNode.cost = currentNode.lmc;
            updateSuccessorVertices(G,Q,currentNode,km);
        else
            currentNode.cost = inf;
            updateVertex(G,Q,currentNode,km);
            updateSuccessorVertices(G,Q,currentNode,km);
        end
    end
    status = ~isinf(G.startNode.cost); %if infinite cost, no path exists CURRENTLY
 
end

%-------------------------------------------------------------------------%
%Environment dynamicity (as sensed by the robot) related function
function modifyEdgeCosts(F,G,Q,O,T,km)   
    
    
    obs = O.senseObstacles(G.startNode.pose);
    modifiedEdges = O.getModifiedEdges(F,G,T,obs);
    
     if (isempty(modifiedEdges))
         return
     end

    for i=1:length(modifiedEdges)
        %nodeIndex = modifiedEdges(i).parent;
        nodeIndex = modifiedEdges(i).child;
        node = G.graphNodes(nodeIndex);
        updateVertex(G,Q,node,km);
    end
    
end
     
%-------------------------------------------------------------------------%
%Plotting functions
function setupPlot(envLB, envUB)
    figure
    clf
    axis equal
    xlim([envLB envUB])
    ylim([envLB envUB])
    hold on
    rectangle('Position',[envLB envLB envUB envUB])
end

%-------------------------------------------------------------------------%
%Saving data functions
function fileCount = saveData(F,G,O,dir,fileCount)
    nodes = G.graphNodes;
    edges = G.graphEdges;
    funnels = F.funnelEdges;  
    obstacles = O.obstacles;
    robotNode = G.startNode;
    save([dir 'iteration_' num2str(fileCount) '.mat'],'nodes','edges','funnels','obstacles','robotNode');
    
    fileCount = fileCount+1;
end