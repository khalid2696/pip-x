
% The MIT License (MIT)
%
% Copyright June, 2019 Michael Otte, University of Maryland
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

classdef heap < handle

    % Copyright 2019 Michael Otte, University of Maryland
    
    % impliments a self-balancing binary search tree version
    % of a min queue (heap)
    % This is a computationally efficient implimentation
    % at least for languages that are quick on branch instructions
    % (i.e., it may not be fast for "small" problem instances in Matlab)

    
    % heapNodes are assumed to be handle types and to have the following fields:
    %    heapIndex (necessary to enable internal removes)
    %    inHeap    (true if in heap, otherwise false)
    
    properties
        capacity         % max nodes the heap can hold before we need to grow it
    	heapCost         % cost values array (the key)
        heapNode         % an array of handles to structs of some valid node type (see notes above)
        indexOfLast      % stores the index of the parent of the last node
        parentOfLast     % used to help swap nodes
        tempCost         % used to help swap nodes
        tempNode         % used to help swap nodes
    end
    methods
        function obj = heap(numElements)
            % numElements is the reserved number of elements that
            % will ever be inserted, the heap will grow as necessary
            % doubling each time it increases in size
            
            % note that things are easier if heap size is a power of 2
            obj.capacity = 2^ceil(log2(numElements));
            
            obj.heapCost = inf(obj.capacity,2);
            obj.heapNode = cell(obj.capacity,1);
            obj.indexOfLast = 0;                   % fixed for 1 indexing
            obj.parentOfLast = 0;                  % fixed for 1 indexing
            obj.tempCost = [inf inf];
            obj.tempNode = nan;

        end
        
        function ret = bubbleUp(obj, n)
        
            % compares a node at index n with its parent, and switches them if the parent's
            % cost is more than the node's cost. Repeats if a switch happens.
            % returns the index where the node ended up.

            tempInd = floor(n/2);                    % fixed for 1 indexing
            while n ~= 1 && obj.compareKey(obj.heapCost(n,:), obj.heapCost(tempInd,:))   % fixed for 1 indexing

                % swap costs 
                obj.tempCost = obj.heapCost(tempInd,:); 
                obj.heapCost(tempInd,:) = obj.heapCost(n,:);
                obj.heapCost(n,:) = obj.tempCost;
      
                % swap graph node pointers
                obj.tempNode = obj.heapNode{tempInd};
                obj.heapNode{tempInd} = obj.heapNode{n};
                obj.heapNode{n} = obj.tempNode;
     
                % update graph node heap index values
                obj.heapNode{tempInd}.heapIndex = tempInd;
                obj.heapNode{n}.heapIndex = n;
   
                % get new node and parent indicies
                n = tempInd;
                tempInd = floor(n/2);                % fixed for 1 indexing
            end
            ret = n;
        end
        
        function bubbleDown(obj, n)
        
            %  compares a node at index n with its children, and switches them if a child's cost
            %  is less than the node's cost. Repeats if a switch happens.

            if n >= obj.capacity/2
               % then it is already in a leaf 
               return
            end
            
            % find child with smallest value
            %if obj.heapCost(2*n) < obj.heapCost(2*n+1)    % fixed for 1 indexing
            if obj.compareKey(obj.heapCost(2*n,:), obj.heapCost(2*n+1,:))
                tempInd = 2*n;                            % fixed for 1 indexing
            else
                tempInd = 2*n+1;                          % fixed for 1 indexing
            end
            
            while n < obj.capacity/2 && obj.compareKey(obj.heapCost(tempInd,:), obj.heapCost(n,:))
                % swap costs 
                obj.tempCost = obj.heapCost(tempInd,:); 
                obj.heapCost(tempInd,:) = obj.heapCost(n,:);
                obj.heapCost(n,:) = obj.tempCost;
                
                %  swap graph node pointers
                obj.tempNode = obj.heapNode{tempInd};
                obj.heapNode{tempInd} = obj.heapNode{n};
                obj.heapNode{n} = obj.tempNode;  
                 
                % update graph node heap index values
                obj.heapNode{tempInd}.heapIndex = tempInd;
                obj.heapNode{n}.heapIndex = n;
      
                % get new node and child indicies
                
                n = tempInd;
   
                if n >= obj.capacity/2
                    % then it is already in a leaf 
                    return
                end
                
                if obj.compareKey(obj.heapCost(2*n,:), obj.heapCost(2*n+1,:))  % fixed for 1 indexing
                    tempInd = 2*n;                          % fixed for 1 indexing
                else
                    tempInd = 2*n+1;                        % fixed for 1 indexing
                end
            end
        end
        
        function push(obj, thisNode, keyValue)
            % add thisNode to the heap, with the key value
            
            if obj.indexOfLast == obj.capacity              % fixed for 1 indexing
                increaseHeapSize(obj);
            end
            if(thisNode.inHeap == false)
                
                obj.indexOfLast = obj.indexOfLast + 1;
                obj.parentOfLast = floor((obj.indexOfLast)/2);     % fixed for 1 indexing
                obj.heapNode{obj.indexOfLast} = thisNode;
                obj.heapCost(obj.indexOfLast,:) = keyValue;
                
                thisNode.heapIndex = obj.indexOfLast;
                thisNode.inHeap = true;
                
                bubbleUp(obj,obj.indexOfLast);
            end
        end
 
        function ret = top(obj)
            
            % returns a handel to the node that is on the top of the heap
        
            if(obj.indexOfLast > 0)                     % fixed for 1 indexing
              ret = obj.heapNode{1};                    % fixed for 1 indexing
            else
              ret = [];
            end
        end
 
        function ret = topKey(obj)
            
            % returns the key of the top value
        
            if(obj.indexOfLast > 0)                     % fixed for 1 indexing
              ret = obj.heapCost(1,:);                  % fixed for 1 indexing
            else
              ret = [inf inf];
            end
        end
        
        
        function ret = pop(obj)
            
            % removes the top valued node from the heap and returns a pointer to it
 
            if obj.indexOfLast <= 0                                % fixed for 1 indexing
                ret = [];
                return
            end
            
            oldTopNode = obj.heapNode{1};                          % fixed for 1 indexing
            
            obj.heapNode{1} = obj.heapNode{obj.indexOfLast};       % fixed for 1 indexing
            obj.heapCost(1,:) = obj.heapCost(obj.indexOfLast,:);       % fixed for 1 indexing
            obj.heapNode{1}.heapIndex = 1;                         % fixed for 1 indexing
            obj.heapNode{obj.indexOfLast} = [];
            obj.heapCost(obj.indexOfLast,:) = [inf inf];
            obj.indexOfLast = obj.indexOfLast - 1;
            obj.parentOfLast = floor((obj.indexOfLast)/2);                % fixed for 1 indexing
            bubbleDown(obj,1);                                     % fixed for 1 indexing
            oldTopNode.inHeap = false;  
            oldTopNode.heapIndex = 0;                              % fixed for 1 indexing
            ret = oldTopNode; 
        end
 
        
        function ret = remove(obj, thisNode)
            
            % removes the particular node from the heap
            % (even if that node is internal to the heap)
            % and then rebalances the heap, returns a pointer
            % to the node that has been removed
            
            if ~thisNode.inHeap
                ret = [];
                return
            end
 
            ind = thisNode.heapIndex;

            obj.heapNode{ind} = obj.heapNode{obj.indexOfLast};
            obj.heapCost(ind,:) = obj.heapCost(obj.indexOfLast,:);
            obj.heapNode{ind}.heapIndex = ind;

            obj.heapNode{obj.indexOfLast} = [];
            obj.heapCost(obj.indexOfLast,:) = [inf inf];
            obj.indexOfLast = obj.indexOfLast - 1;
            obj.parentOfLast = floor((obj.indexOfLast)/2);  % fixed for 1 indexing
 
            ind = bubbleUp(obj,ind);
            bubbleDown(obj,ind);

            thisNode.inHeap = false;
            thisNode.heapIndex = 0;                  % fixed for 1 indexing
            ret = thisNode;
        end
     
        
        function update(obj, thisNode, keyValue)
            
            % updates the position of the particular node within the heap
            % to reflect the new key value
            % (even if that node is internal to the heap)
            % and then rebalances the heap)
            % NOTE: this will insert the node if it is not in the heap already

            % we'll just do the easy way
            if thisNode.inHeap                
                remove(obj, thisNode);
            end
            push(obj,thisNode, keyValue);
        end

        function show(obj)
            % displays the size, capacity, and heap key values
            
            disp([num2str(obj.indexOfLast) ' of ' num2str(obj.capacity) ':']);
            disp(obj.heapCost(1:obj.indexOfLast,:)) 
        end

        function ret = isEmpty(obj)
           % returns true if the heap is empty
           
           if obj.indexOfLast == 0
               ret = true;
           else
               ret = false; 
           end  
        end
        
        function ret = correct(obj)
            % returns true if heap is good, false if bad, also prints a command line message
            ret = true;
            for i = 2:obj.indexOfLast
                if obj.compareKey(obj.heapCost(i,:), obj.heapCost(floor(i/2),:)) || obj.heapNode{i}.heapIndex ~= i
                    ret = false;
                    error('There is a problem with the heap');
                end
            end
            if ret
               disp('The heap is OK');
            end
        end

        function increaseHeapSize(obj)
            % increases the heap size by a factor of two
            % this should be used sparingly since it will take a while each
            % time it is run. Best practice is to pre-allocate space instead 
            % of resizing on the fly 

            oldCapacity = obj.capacity;
            obj.capacity = obj.capacity * 2;
            if(obj.capacity < 128) %the minimum size is 128
              obj.capacity = 128;
            end
            
            newHeapCost = inf(obj.capacity,2);
            newHeapCost(1:oldCapacity,:) = obj.heapCost(1:oldCapacity,:);          
            obj.heapCost = newHeapCost;
            
            newHeapNode = cell(obj.capacity,1);
            newHeapNode(1:oldCapacity) = obj.heapNode(1:oldCapacity); 
            obj.heapNode = newHeapNode;
        end
        
        function ret = cleanHeapIntoCellArray(obj)
            % removes all nodes from the heap, but places each in a cell array
            % and returns it
            ret = cell(obj.indexOfLast,1);
            i = 0;
            while obj.indexOfLast > 0
               i = i+1;
               ret{i} = pop(obj);
            end
        end
        
        %returns TRUE if key1 is SMALLER than key2
        %note that it follows a lexographic-type comparison
        function status = compareKey(obj,key1, key2)
            status = false;
            if key1(1) < key2(1)
                status = true;
            elseif key1(1) == key2(1)
                if key1(2) < key2(2)
                    status = true;
                end
            end
        end
                
    end
end

