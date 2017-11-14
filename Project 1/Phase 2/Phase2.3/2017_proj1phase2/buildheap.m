function [heap,indexarray] = buildheap(heap,value,indexarray,nodevalue)
heap=[heap,value];
indexarray=[indexarray,nodevalue];
[heap,indexarray]=heapifyup(heap,numel(heap),indexarray);

end
