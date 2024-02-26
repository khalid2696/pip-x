classdef KDTreeNode < handle

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

    % this is a node with minimum parameters that can be used in the kdTree

    % Copyright (c) January, 2014 Michael Otte, MIT (original Julia Version)
    % Copyright 2019 Michael Otte, University of Maryland (Matlab Version)
     
        
    properties
          kdInTree      % Bool, set to true if this node is in the kd-tree
          kdParentExist % Bool, set to true if parent in the tree is used
          kdChildLExist % Bool, set to true if left child in the tree is used
          kdChildRExist % Bool, set to true if right child in the tree is used

          % data used for heap in KNN-search
          heapIndex     % Int, named such to allow the use of default heap functions
          inHeap        % Bool, ditto
          data          % Float64, ditto, this will hold the distance

          % more data used for KD Tree
          position      % dX1 array where d is the dimesnions of the space
          kdSplit       % Int, the dimension used for splitting at this node 
          kdParent      % KDTreeNode, parent in the tree
          kdChildL      % KDTreeNode, left child in the tree
          kdChildR      % KDTreeNode, right child in the tree
          
          % a universal paylaod for carrying something else
          % e.g., a handle to some other data strucute like a graph node
          payload
    end
    methods
        % constructor
        function obj = KDTreeNode(position)
            obj.kdInTree = false;
            obj.kdParentExist = false;
            obj.kdChildLExist = false;
            obj.kdChildRExist = false;

            % data used for heap in KNN-search
            obj.heapIndex = -1;
            obj.inHeap = false;
            obj.data = -inf;

            % more data used for KD Tree
            obj.position = position;
            obj.kdSplit = -inf;
            
            % these get set later:
            % obj.kdParent 
            % obj.kdChildL 
            % obj.kdChildR
          
        end
    end
end

