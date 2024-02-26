% The MIT License (MIT)
%
% Copyright June, 2019 Michael Otte, Universtiy of Maryland
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
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.


% note that this implimentaion of the KD tree should not be used for cartesian spaces
% in practice (since I've written a faster implimentation, at least in Julia, that
% saves dist squared instad of distance for that particular case)
%
% also note that, by default, the space is assumed to be R^d
% however, it is also possible to allow the space to be some cartesian product of
% R^n and S^m subspaces, more codeing is required if other subspaces are desired,
% e.g. quarternions. Make sure to use an approperiate distance function.
% The particular topology of the space is defined in KDTree{T},
% see notes. The way that wrapping subspaces are handles, currently, is to run
% a check with the original point, and then to run an additional run 3^(w) - 1
% checks, where w is the number of wrapping dimensions, for each possibility of
% wrapping the querery point around some/all of the wrappable dimensions.
%
% in practice, many of these can be skipped if the distance from the wrapped point
% to the closest point in the unwrapped space > the distance to the nearest point
% found so far. FINALLY NOTE that the kd-tree distance function used to help
% find nearest neighbors should actually treat S dimesnions as R dimensions, since
% ghost points are used for wrapping!! so you'll need a swperate kd-tree distance
% function than the one used in the rest of your code where S dimension are
% wrapped inside the distance function.
%
% knn search does not yet work for cases of wrapped dimensions
% ranged search works for cases of wrapped dimensions

classdef KDTree < handle
    % Copyright (c) January, 2014 Michael Otte, MIT (original Julia Version)
    % Copyright 2019 Michael Otte, University of Maryland (Matlab Version)
    
    
    % impliments a kd tree
    properties
        d                 % Int, the number of dimensions in the space
        distanceFunction  % Function, the distance function to use: f(poseA, poseB)
        treeSize          % Int, the number of nodes in the KD-Tree
        
        numWraps          % Int, the total number of dimensions that wrap
        wraps             % Array{Int}, a vector of length d containing a list of
        % all the dimensions that wrapAround
        
        wrapPoints        % Array{Float64},  space is assumed to start at 0 and end at
        % wrapPoints[i] along dimension wraps[i]
        
        root              % the root node
    end
    methods
        
        % constructor
        function obj = KDTree(numDimensions, distanceFunction)
            obj.d = numDimensions;
            obj.distanceFunction = distanceFunction;
            obj.treeSize = 0;
            obj.numWraps = 0;    % gets reset later if wraps are used
        end
        
        % this is the "constructor" to use if we care about dimensions that
        % wrap around (not all dimensions need to wrap)
        function setWrapDimensions(obj, wraps, wrapPoints)
            obj.numWraps = length(wraps);
            obj.wraps = wraps;
            obj.wrapPoints = wrapPoints;
        end
        
        
        % inserts a new node into the tree
        function kdInsert(obj, node)
            if node.kdInTree
                return
            end
            node.kdInTree = true;
            
            if obj.treeSize == 0
                obj.root = node;
                obj.root.kdSplit = 1;
                obj.treeSize = 1;
                return
            end
            
            % figure out where to put this node
            parent = obj.root;
            while true
                if node.position(parent.kdSplit) < parent.position(parent.kdSplit)
                    % traverse tree to the left
                    if ~parent.kdChildLExist
                        % the node gets inserted as the left child of the parent
                        parent.kdChildL = node;
                        parent.kdChildLExist = true;
                        break
                    end
                    
                    parent = parent.kdChildL;
                    continue
                else
                    % traverse tree to the right
                    if ~parent.kdChildRExist
                        % the node gets inserted as the right child of the parent
                        parent.kdChildR = node;
                        parent.kdChildRExist = true;
                        break
                    end
                    
                    parent = parent.kdChildR;
                    continue
                end
            end
            
            node.kdParent = parent;
            node.kdParentExist = true;
            if parent.kdSplit == obj.d
                node.kdSplit = 1;
            else
                node.kdSplit = parent.kdSplit + 1;
            end
            obj.treeSize = obj.treeSize +1;
        end
        
        
        
        % this is more convienient to use when we want to use the payload
        % field in a generic KDtreeNode to store what is being inserted 
        % instead of assuming that what is being inserted will have all
        % the correct fields. This assumes that for typing
        % payloadNode < handle so that we get a reference and not the 
        % actual thing. Assumes that payloadNode does have the required
        % position field
        function kdInsertAsPayload(obj, payloadNode)
            kdtnode = KDTreeNode(payloadNode.pose); %changing it to pose instead of position
            kdtnode.payload = payloadNode;
            kdInsert(obj, kdtnode);
        end
        
        
        
        % this returns a matrix where each row is the position of a node
        % that is stored in the kd-tree, it is useful for plotting what
        % is going on. if treeForm = true, then we save nodes as we go both
        % up and down the tree, which makes visualizing the tree easier
        % otherwise we only save nodes once (on the way down).
        function positions = extractAllPositions(obj, treeForm)
            
            if treeForm
                numToStoreBound = obj.treeSize*3;
            else
                numToStoreBound = obj.treeSize;
            end
            
            positions = nan(numToStoreBound, obj.d);
            
            searchStack = char(ones(obj.treeSize,1));   % this will help us walk the tree
            ssp = 1;                                % pointer to position in searchStack
            searchStack(ssp) = 'L';                 % record that we want to next look at the left child
            
            % normally we'd write this recursively, but malab can barf on
            % deep recursion, so we'll walk the tree structure instead
            
            i = 1;
            currentNode = obj.root;
            positions(i,:) = currentNode.position;
            
            while ssp > 0
                if searchStack(ssp) == 'L'
                    % if we're going left (and down)
                    
                    % go left (and down) if possible
                    if currentNode.kdChildLExist
                        
                        % a left child exists, so go that way
                        currentNode = currentNode.kdChildL;
                        i = i+1;
                        positions(i,:) = currentNode.position;
                        ssp = ssp + 1;
                        searchStack(ssp) = 'L';  % search left from the child
                    else
                        % no left child exists
                        % so record that we next want to look right
                        searchStack(ssp) = 'R';
                    end
                elseif searchStack(ssp) == 'R'
                    % we're going right (and down)
                    
                    % go right (and down) if possible
                    if currentNode.kdChildRExist
                        
                        % a right child exists, so go that way
                        currentNode = currentNode.kdChildR;
                        i = i+1;
                        positions(i,:) = currentNode.position;
                        ssp = ssp + 1;
                        searchStack(ssp) = 'L';   % search left from the child
                    else
                        % no right child exists, and we've already looked left
                        
                        % so go up until we can go right again
                        while ssp > 0 && searchStack(ssp) == 'R'
                            ssp = ssp - 1;
                            currentNode = currentNode.kdParent;
                            
                            if ssp > 0
                                if treeForm
                                    i = i+1;
                                    positions(i,:) = currentNode.position;
                                end
                            end
                        end
                        
                        if ssp == 0
                            % reached root
                            break
                        end
                        searchStack(ssp) = 'R';
                    end
                end
            end
            
            if treeForm
                positions = positions(1:i,:);
            end
        end
        
        
        
        % ############################### Nearest ##################################
        
        
        % returns the nearest node to queryPoint in the subtree starting at root
        % and also its distance, it takes also takes a suggestion for a
        % possible closest node (and uses that if it is best)
        function [ret1, ret2] = kdFindNearestInSubtree(obj, distanceFunction, root, queryPoint, suggestedClosestNode, suggestedClosestDist)
            
            % walk down the tree as if the node would be inserted
            parent = root;
            currentClosestNode = suggestedClosestNode;
            currentClosestDist = suggestedClosestDist;
            while true
                if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit)
                    % traverse tree to the left
                    if ~parent.kdChildLExist
                        % the queryPoint would be inserted as the left child of the parent
                        break
                    end
                    parent = parent.kdChildL;
                    continue
                else
                    % traverse tree to the right
                    if ~parent.kdChildRExist
                        % the queryPoint would be inserted as the right child of the parent
                        break
                    end
                    parent = parent.kdChildR;
                    continue
                end
            end
            
            newDist = distanceFunction(queryPoint,parent.position) ;
            if newDist < currentClosestDist
                currentClosestNode = parent;
                currentClosestDist = newDist;
            end
            
            % now walk back up the tree (will break out when done)
            while true
                % now check if there could possibly be any closer nodes on the other
                % side of the parent, if not then check grandparent etc.
                
                parentHyperPlaneDist = (queryPoint(parent.kdSplit) - parent.position(parent.kdSplit));
                
                if parentHyperPlaneDist > currentClosestDist
                    % then there could not be any closer nodes on the other side of the parent
                    % (and the parent itself is also too far away
                    
                    if parent == root
                        % the parent is the root and we are done
                        ret1 = currentClosestNode;
                        ret2 = currentClosestDist;
                        return
                    end
                    
                    parent = parent.kdParent;
                    continue
                end
                
                % if we are here, then there could be a closer node on the other side of the
                % parent (including the parent itself)
                
                % first check the parent itself (if it is not already the closest node)
                if currentClosestNode ~= parent
                    newDist = distanceFunction(queryPoint,parent.position);
                    if newDist < currentClosestDist
                        currentClosestNode = parent;
                        currentClosestDist = newDist;
                    end
                end
                
                % now check on the other side of the parent
                if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit) && parent.kdChildRExist
                    % queryPoint is on the left side of the parent, so we need to look
                    % at the right side of it (if it exists)
                    
                    % find right subtree dist
                    [Rnode, Rdist] = kdFindNearestInSubtree(obj, distanceFunction, parent.kdChildR, queryPoint, currentClosestNode, currentClosestDist);
                    
                    if Rdist < currentClosestDist
                        currentClosestDist = Rdist;
                        currentClosestNode = Rnode;
                    end
                    
                elseif parent.position(parent.kdSplit) <= queryPoint(parent.kdSplit) && parent.kdChildLExist
                    % queryPoint is on the right side of the parent, so we need to look
                    % at the left side of it (if it exists)
                    
                    % find left subtree dist
                    [Lnode, Ldist] = kdFindNearestInSubtree(obj, distanceFunction, parent.kdChildL, queryPoint, currentClosestNode, currentClosestDist);
                    if Ldist < currentClosestDist
                        currentClosestDist = Ldist;
                        currentClosestNode = Lnode;
                    end
                end
                
                if parent == root
                    % the parent is the root and we are done
                    
                    ret1 = currentClosestNode;
                    ret2 = currentClosestDist;
                    return
                end
                
                parent = parent.kdParent;
            end
        end
        
        % returns the nearest node to queryPoint and also its distance
        %ret 1 - nearest node
        %ret 2 - distance
        function [ret1, ret2] = kdFindNearest(obj, queryPoint)
            % initial search (only search if the space does not wrap around)
            distToRoot = obj.distanceFunction(queryPoint, obj.root.position);
            [Lnode, Ldist] = kdFindNearestInSubtree(obj, obj.distanceFunction, obj.root, queryPoint, obj.root, distToRoot);
            
            if obj.numWraps > 0
                % if dimensions wrap around, we need to search vs. identities (ghosts)
                
                pointIterator = ghostPointIterator(obj, queryPoint);
                while true
                    thisGhostPoint = getNextGhostPoint(pointIterator, Ldist);
                    if isempty(thisGhostPoint)
                        break
                    end
                    
                    % now see if any points in the space are closer to this ghost
                    distGhostToRoot = obj.distanceFunction(thisGhostPoint, obj.root.position);
                    [thisLnode, thisLdist] = kdFindNearestInSubtree(obj, obj.distanceFunction, obj.root, thisGhostPoint, obj.root, distGhostToRoot);
                    
                    if thisLdist < Ldist
                        % found closer point
                        Ldist = thisLdist;
                        Lnode = thisLnode;
                    end
                end
            end
            
            ret1 = Lnode;
            ret2 = Ldist;
            return
        end
        
        
        
        % returns the nearest node to queryPoint and also its distance
        % this version is used when what we actually want to use is stored
        % in the payload of a standard KDTreeNode
        function [ret1, ret2] = kdFindNearestPayload(obj, queryPoint)
            [kdtn, ret2] = kdFindNearest(obj, queryPoint);
            ret1 = kdtn.payload;
        end
        
        
        % returns the nearest node to queryPoint in the subtree starting at root
        % and also its distance, it takes also takes a suggestion for a
        % possible closest node (and uses that if it is best)
        function [ret1, ret2] = kdFindNearestInSubtreeWithGuess(obj, distanceFunction, root, queryPoint, suggestedClosestNode, suggestedClosestDist)
            
            % walk down the tree as if the node would be inserted
            parent = suggestedClosestNode.kdParent;
            currentClosestNode = suggestedClosestNode;
            currentClosestDist = suggestedClosestDist;
            while true
                if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit)
                    % traverse tree to the left
                    if ~parent.kdChildLExist
                        % the queryPoint would be inserted as the left child of the parent
                        break
                    end
                    parent = parent.kdChildL;
                    continue
                else
                    % traverse tree to the right
                    if ~parent.kdChildRExist
                        % the queryPoint would be inserted as the right child of the parent
                        break
                    end
                    parent = parent.kdChildR;
                    continue
                end
            end
            
            newDist = distanceFunction(queryPoint, parent.position);
            if newDist < currentClosestDist
                currentClosestNode = parent;
                currentClosestDist = newDist;
            end
            
            % now walk back up the tree (will break out when done)
            while true
                % now check if there could possibly be any closer nodes on the other
                % side of the parent, if not then check grandparent etc.
                
                parentHyperPlaneDist = (queryPoint(parent.kdSplit) - parent.position(parent.kdSplit));
                
                if parentHyperPlaneDist > currentClosestDist
                    % then there could not be any closer nodes on the other side of the parent
                    % (and the parent itself is also too far away
                    
                    if parent == root
                        % the parent is the root and we are done
                        ret1 = currentClosestNode;
                        ret2 = currentClosestDist;
                        return
                    end
                    
                    parent = parent.kdParent;
                    continue
                end
                
                % if we are here, then there could be a closer node on the other side of the
                % parent (including the parent itself)
                
                % first check the parent itself (if it is not already the closest node)
                if currentClosestNode ~= parent
                    newDist = distanceFunction(queryPoint,parent.position);
                    if newDist < currentClosestDist
                        currentClosestNode = parent;
                        currentClosestDist = newDist;
                    end
                end
                
                % now check on the other side of the parent
                if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit) && parent.kdChildRExist
                    % queryPoint is on the left side of the parent, so we need to look
                    % at the right side of it (if it exists)
                    
                    % find right subtree dist
                    
                    [Rnode, Rdist] = kdFindNearestInSubtree(obj, distanceFunction, parent.kdChildR, queryPoint, currentClosestNode, currentClosestDist);
                    
                    if Rdist < currentClosestDist
                        currentClosestDist = Rdist;
                        currentClosestNode = Rnode;
                    end
                    
                elseif parent.position(parent.kdSplit) <= queryPoint(parent.kdSplit) && parent.kdChildLExist
                    % queryPoint is on the right side of the parent, so we need to look
                    % at the left side of it (if it exists)
                    
                    % find left subtree dist
                    
                    [Lnode, Ldist] = kdFindNearestInSubtree(obj, distanceFunction, parent.kdChildL, queryPoint, currentClosestNode, currentClosestDist);
                    if Ldist < currentClosestDist
                        currentClosestDist = Ldist;
                        currentClosestNode = Lnode;
                    end
                end
                
                if parent == root
                    % the parent is the root and we are done
                    % need to do one last check vs the root
                    thisDist = distanceFunction(queryPoint,parent.position);
                    if thisDist < currentClosestDist
                        ret1 = parent;
                        ret2 = thisDist;
                        return
                    end
                    
                    ret1 = currentClosestNode;
                    ret2 = currentClosestDist;
                    return
                end
                
                parent = parent.kdParent;
            end
        end
        
        % returns the nearest node to queryPoint and also its distance squared,
        % instead of starting at the root, it starts at guess
        function [ret1, ret2] = kdFindNearestWithGuess(obj, queryPoint, guess)
            distToGuess = obj.distanceFunction(queryPoint, guess.position);
            if guess == obj.root
                [ret1, ret2] = kdFindNearestInSubtree(obj, obj.distanceFunction, obj.root, queryPoint, obj.root, distToGuess);
                return
            end
            
            [Lnode, Ldist] = kdFindNearestInSubtreeWithGuess(obj, obj.distanceFunction, obj.root, queryPoint, guess, distToGuess);
            
            if obj.numWraps > 0
                % if dimensions wrap around, we need to search vs. identities (ghosts)
                
                pointIterator = ghostPointIterator(obj, queryPoint);
                while true
                    thisGhostPoint = getNextGhostPoint(pointIterator, Ldist);
                    if isempty(thisGhostPoint)
                        break
                    end
                    
                    % now see if any points in the space are closer to this ghost
                    distGhostToGuess = obj.distanceFunction(thisGhostPoint, guess.position);
                    [thisLnode, thisLdist] = kdFindNearestInSubtreeWithGuess(obj, obj.distanceFunction, obj.root, thisGhostPoint, guess, distGhostToGuess);
                    
                    if thisLdist < Ldist
                        % found closer point
                        Ldist = thisLdist;
                        Lnode = thisLnode;
                    end
                end
            end
            ret1 = Lnode;
            ret2 = Ldist;
            return
        end
        
        
        
        % same as above, but
        % this version is used when what we actually want to use is stored
        % in the payload of a standard KDTreeNode
        function [ret1, ret2] = kdFindNearestWithGuessPayload(obj, queryPoint, guess)
            [kdtn, ret2] = kdFindNearestWithGuess(obj, queryPoint, guess);
            ret1 = kdtn.payload;
        end
        
        
        
        
        
        %############################## K Nearest #############################
        
        % adds the node to the heap IF there is space in the current heap without growing
        % past k, othewise the curent top is removed first, returns the current top
        % note this assumes a min heap (which we hijack later to be a max heap by sending in negative costs)
        function ret = addToKNNHeap(obj, H, thisNode, key, k)
            if thisNode.inHeap
                ret = top(H);
                return
            elseif H.indexOfLast < k
                % just insert
                thisNode.data = key;
                push(H, thisNode, thisNode.data);
            else
                topNode = top(H);
                if topNode.data < key
                    pop(H);
                    thisNode.data = key; 
                    push(H, thisNode, thisNode.data);
                end
            end
            ret = top(H);
            return
        end
        
        
        % finds the K nearest nodes to queryPoint in the subtree starting at root
        % note that this data is stored in the nearestHeap, and the heap may also contain
        % nodes before this function is called explicitly returns the node of the nearest
        % set that is FURTHEREST from the querery along with its distance
        % assumes that there is at least one node in the heap to begin with
        % IF this is the first call to this function (e.g., from kdFindKNearest) then
        % it is also assumed that a dummy node has been added that has an Inf key value
        % this makes things easier with checking that all K slots are used during the
        % recursion
        function ret = kdFindKNearestInSubtree(obj, distanceFunction, root, k, queryPoint, nearestHeap)
        
          % walk down the tree as if the node would be inserted
          parent = root;
          currentWorstClosestNode = top(nearestHeap);
          currentWorstClosestDist = -currentWorstClosestNode.data;  % we're hijacking a min queue to be a max queue, hence the negative
          while true
            if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit)
              % traverse tree to the left
              if ~parent.kdChildLExist
                % the queryPoint would be inserted as the left child of the parent
                break
              end
              parent = parent.kdChildL;
              continue
            else
              % traverse tree to the right
              if ~parent.kdChildRExist
                % the queryPoint would be inserted as the right child of the parent
                break
              end
              parent = parent.kdChildR;
              continue
            end
          end
        
          newDist = distanceFunction(queryPoint,parent.position);
          if newDist < currentWorstClosestDist
            currentWorstClosestNode = addToKNNHeap(obj, nearestHeap, parent, -newDist, k);  % we're hijacking a min queue to be a max queue, hence the negative
            currentWorstClosestDist = -currentWorstClosestNode.data;                        % we're hijacking a min queue to be a max queue, hence the negative
          end
        
          % now walk back up the tree (will break out when done)
          while true
        
            % now check if there could possibly be any closer nodes on the other
            % side of the parent, if not then check grandparent etc.
        
            parentHyperPlaneDist = (queryPoint(parent.kdSplit) - parent.position(parent.kdSplit));
        
            if parentHyperPlaneDist >  currentWorstClosestDist
              % then there could not be any closer nodes on the other side of the parent
              % (and the parent itself is also too far away
        
              if parent == root
                % the parent is the root and we are done
                ret = currentWorstClosestNode;
                return 
              end
        
              parent = parent.kdParent;
              continue
            end
        
            % if we are here, then there could be a closer node on the other side of the
            % parent (including the parent itself)
        
            % first check the parent itself (if it is not already one of the closest nodes)
            if ~parent.inHeap
              newDist = distanceFunction(queryPoint,parent.position);
              if newDist < currentWorstClosestDist
                currentWorstClosestNode = addToKNNHeap(obj, nearestHeap, parent, -newDist, k);   % we're hijacking a min queue to be a max queue, hence the negative
                currentWorstClosestDist = -currentWorstClosestNode.data;                         % we're hijacking a min queue to be a max queue, hence the negative
              end
            end
        
            % now check on the other side of the parent
            if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit) && parent.kdChildRExist
              % queryPoint is on the left side of the parent, so we need to look
              % at the right side of it (if it exists)
              currentWorstClosestNode = kdFindKNearestInSubtree(obj, distanceFunction, parent.kdChildR, k, queryPoint, nearestHeap);
              currentWorstClosestDist = -currentWorstClosestNode.data; % we're hijacking a min queue to be a max queue, hence the negative
            elseif parent.position(parent.kdSplit) <= queryPoint(parent.kdSplit) && parent.kdChildLExist
              % queryPoint is on the right side of the parent, so we need to look
              % at the left side of it (if it exists)
              currentWorstClosestNode = kdFindKNearestInSubtree(obj, distanceFunction, parent.kdChildL, k, queryPoint, nearestHeap);
              currentWorstClosestDist = -currentWorstClosestNode.data; % we're hijacking a min queue to be a max queue, hence the negative
            end
        
            if parent == root
              % the parent is the root and we are done
              ret = currentWorstClosestNode;
              return 
            end
        
            parent = parent.kdParent;
          end
        end
        
        
        % returns the K nearest nodes to queryPoint and also their distance
        % (note that they are not sorted, but they are in (reverse) heap order)
        function ret = kdFindKNearest(obj, k, queryPoint)
          H = heap(k);
        
          % insert root node in heap
          rootKey = obj.distanceFunction(queryPoint, obj.root.position); % we're hijacking a min queue to be a max queue, hence the negative
          % push(H, obj.root, obj.root.data)
          addToKNNHeap(obj, H, obj.root, -rootKey, k) % we're hijacking a min queue to be a max queue, hence the negative

          
          % insert a dummy node in the heap with inf key
          % note, this is necessary for the algorithm to work correctly
          dummyNode =  KDTreeNode([]);
          % dummyNode.data = -Inf;               % negative since we're hijacking a min queue for a max queue 
          % push(H, dummyNode, dummyNode.data);
          addToKNNHeap(obj, H, dummyNode, -Inf, k) % we're hijacking a min queue to be a max queue, hence the negative

        
          % find k nearest neighbors
          kdFindKNearestInSubtree(obj, obj.distanceFunction, obj.root, k, queryPoint, H)
        
          if obj.numWraps > 0
            error('knn search has not been implimented for wrapped space')
          end        
        
          % remove the dummy node if still there (guarenteed to be on top, due to Inf key)
          topNode = top(H);
          if topNode == dummyNode
            pop(H);
          end
        
          ret = cleanHeapIntoCellArray(H);
          return 
        end
        
        
        % same as above, but
        % this version is used when what we actually want to use is stored
        % in the payload of a standard KDTreeNode
        function ret = kdFindKNearestPayload(obj, k, queryPoint)
            ret = kdFindKNearest(obj, k, queryPoint);
            for i = 1:length(ret)
              ret{i} = ret{i}.payload;
            end
        end
        
        
        
        
       function ret = positionArrayFromCellArray(obj, C)
            % used mostly for visualizing what is going on with the kd-tree
            % stores all positions in nodes stored in a cell array into
            % a single noremal array that can easily be plotted
            ret = nan(length(C), obj.d);
            for i = 1:length(C)
               ret(i,:) = C{i}.position(:); 
            end
        end
        
        
        %############################ Within Range #############################
        
        
        
        
        
        % adds the node to the list if it is not already there
        function addToRangeList(obj, S, thisNode, key)
            if thisNode.inHeap
                return
            end
            thisNode.inHeap = true;      % note that inHeap is a misnomer since this is a list
            push(S, thisNode, key);
        end
        
        % pops the range list
        function ret = popFromRangeList(obj, S)
          thisNode = pop(S);
          thisNode.inHeap = false;       % note that inHeap is a misnomer since this is a list
        
          ret = thisNode;
          return 
        end

        % empty the range list
        function ret = emptyRangeListIntoCellArray(obj, S)
            
          ret = cell(S.last,1);
            
          i = 0;
          while S.last > 0
            i = i+1;
            ret{i} = pop(S);
            ret{i}.inHeap = false;  % note that inHeap is a misnomer since this is a list
          end
        end
        
        
        % helpful for plotting and debugging, should probably not be used for anything else
        function ret = emptyRangeListToPositionArray(obj, S)
          
          ret = zeros(S.last, obj.d);
          i = 0;
          while S.last > 0
            thisNode= pop(S);
            thisNode.inHeap = false;  % note that inHeap is a misnomer since this is a list
            
            i = i+1;
            ret(i,:) = thisNode.position;
          end
        end
        
        
        % finds all nodes within range of the queryPoint in the subtree starting at root
        % and also their distance squared, note that this data is stored in the nodeList
        % the nodeList may also contain nodes before this function is called
        % note, we are using a filoGrowableQ for the nodeList
        function kdFindWithinRangeInSubtree(obj, distanceFunction, root, range, queryPoint, nodeList)
        
          % walk down the tree as if the node would be inserted
          parent = root;
          while true
            if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit)
              % traverse tree to the left
              if ~parent.kdChildLExist
                % the queryPoint would be inserted as the left child of the parent
                break
              end
              parent = parent.kdChildL;
              continue
            else
              % traverse tree to the right
              if ~parent.kdChildRExist
                % the queryPoint would be inserted as the right child of the parent
                break
              end
              parent = parent.kdChildR;
              continue
            end
          end
        
          newDist = distanceFunction(queryPoint,parent.position);
          if newDist < range
            addToRangeList(obj, nodeList, parent, newDist)
          end
        
          % now walk back up the tree (will break out when done)
          while true
        
            % now check if there could possibly be any nodes on the other
            % side of the parent within range, if not then check grandparent etc.
        
            parentHyperPlaneDist = queryPoint(parent.kdSplit) - parent.position(parent.kdSplit);
        
            if parentHyperPlaneDist > range
              % then there could not be any closer nodes within range on the other
              % side of the parent (and the parent itself is also too far away
        
              if parent == root
                % the parent is the root and we are done
                return
              end
        
              parent = parent.kdParent;
              continue
            end
        
            % if we are here, then there could be a closer node on the other side of the
            % parent (including the parent itself) that is within range
        
            % first check the parent itself (if it is not already one of the closest nodes)
            if ~parent.inHeap    % note that inHeap is a misnomer since this is a list
              newDist = distanceFunction(queryPoint,parent.position);
              if newDist < range
                addToRangeList(obj, nodeList, parent, newDist);
              end
            end
        
            % now check on the other side of the parent
            if queryPoint(parent.kdSplit) < parent.position(parent.kdSplit) && parent.kdChildRExist
              % queryPoint is on the left side of the parent, so we need to look
              % at the right side of it (if it exists)
              kdFindWithinRangeInSubtree(obj, distanceFunction, parent.kdChildR, range, queryPoint, nodeList)
            elseif parent.position(parent.kdSplit) <= queryPoint(parent.kdSplit) && parent.kdChildLExist
              % queryPoint is on the right side of the parent, so we need to look
              % at the left side of it (if it exists)
              kdFindWithinRangeInSubtree(obj, distanceFunction, parent.kdChildL, range, queryPoint, nodeList)
            end
        
            if parent == root
              % the parent is the root and we are done
              return
            end
        
            parent = parent.kdParent;
          end
        end
        
        
        % returns all nodes within range of queryPoint and also their distance
        % they are returned in a fifoGrowableQ
        function ret = kdFindWithinRange(obj, range, queryPoint)
        
          % make fifoGrowableQ to store nodes in, we'll start with space
          % for 100 nodes, but it will grow (doubling in size) as necessary
          L = filoGrowableQ(100);
        
          % insert root node in list if it is within range
          distToRoot = obj.distanceFunction(queryPoint, obj.root.position);
          if distToRoot <= range
            addToRangeList(obj, L, obj.root, distToRoot);
          end
        
          % find nodes within range
          kdFindWithinRangeInSubtree(obj, obj.distanceFunction, obj.root, range, queryPoint, L);
        
          if obj.numWraps > 0
            % if dimensions wrap around, we need to search vs. identities (ghosts)
        
            pointIterator = ghostPointIterator(obj, queryPoint);
            while true
              thisGhostPoint = getNextGhostPoint(pointIterator, range);
              if isempty(thisGhostPoint)
                break
              end
        
              % now see if any points in the space are closer to this ghost
              kdFindWithinRangeInSubtree(obj, obj.distanceFunction, obj.root, range, thisGhostPoint, L)
            end
          end
        
          ret = emptyRangeListIntoCellArray(obj, L);
          return
        end
        
        
        
        % same as above, but
        % this version is used when what we actually want to use is stored
        % in the payload of a standard KDTreeNode
        function ret = kdFindWithinRangePayload(obj, range, queryPoint)
            ret = kdFindWithinRange(obj, range, queryPoint);
            for i = 1:length(ret)
              ret{i} = ret{i}.payload;
            end
        end 
        
         
        
        % returns all nodes within range of queryPoint and also their distance
        % they are returned in a fifoGrowableQ 
        % the list to be used is passed in, so that additional points can be added
        % to it (e.g., if we want to have one list containing the points that
        % are close to a couple of different points X1 .. Xn, then call this
        % for X2 ... Xn after first calling kdFindWithinRange for X1
        function ret =  kdFindMoreWithinRange(obj, range, queryPoint, L)
        
          % insert root node in list if it is within range
          distToRoot = tree.distanceFunction(queryPoint, tree.root.position);
          if distToRoot <= range
            addToRangeList(obj, L, obj.root, distToRoot)
          end
        
          % find nodes within range
          kdFindWithinRangeInSubtree(obj, obj.distanceFunction, obj.root, range, queryPoint, L)
        
          if obj.numWraps > 0
            % if dimensions wrap around, we need to search vs. identities (ghosts)
        
            pointIterator = ghostPointIterator(obj, queryPoint);
            while true
              thisGhostPoint = getNextGhostPoint(pointIterator, range);
              if isempty(thisGhostPoint)
                break
              end
        
              % now see if any points in the space are closer to this ghost
              kdFindWithinRangeInSubtree(obj, obj.distanceFunction, obj.root, range, thisGhostPoint, L)
            end
          end
        
          ret = emptyRangeListIntoCellArray(obj, L);
          return
        end
          
        
    end
end

