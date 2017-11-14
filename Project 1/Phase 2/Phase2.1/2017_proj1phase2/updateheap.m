function[heap,indexarray]=updateheap(heap,indexarray,newvalue,index)
if heap(index)< newvalue
    heap(index)=newvalue;
    [heap, indexarray]=heapifydown(heap,index,indexarray);
else
    if heap(index)> newvalue
           heap(index)=newvalue;
    [heap, indexarray]=heapifyup(heap,index,indexarray);
    end
end
end
