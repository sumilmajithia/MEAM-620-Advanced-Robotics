function[heap,indexarray,nodeindexarray]=updateheap(heap,indexarray,newvalue,index,nodeindexarray)
if heap(index)< newvalue
    heap(index)=newvalue;
    [heap, indexarray,nodeindexarray]=heapifydown(heap,index,indexarray,nodeindexarray);
else
    if heap(index)> newvalue
           heap(index)=newvalue;
    [heap, indexarray,nodeindexarray]=heapifyup(heap,index,indexarray,nodeindexarray);
    end
end
end
