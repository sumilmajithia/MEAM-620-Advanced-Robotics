function [heap,indexarray,nodeindexarray] = buildheap(heap,value,indexarray,nodevalue,nodeindexarray)
heap=[heap,value];
indexarray=[indexarray,nodevalue];
nodeindexarray(nodevalue)=numel(heap);
[heap,indexarray,nodeindexarray]=heapifyup(heap,numel(heap),indexarray,nodeindexarray);

end
