function [heap,minvalue,index,indexarray,nodeindexarray]=extractmin(heap,indexarray,nodeindexarray)
minvalue=heap(1);
index=indexarray(1);
temp=heap(end);
heap(end)=[];
heap(1)=temp;
temp=indexarray(end);
indexarray(end)=[];
indexarray(1)=temp;
nodeindexarray(temp)=1;
nodeindexarray(index)=0;

[heap,indexarray,nodeindexarray]=heapifydown(heap,1,indexarray,nodeindexarray);
end


