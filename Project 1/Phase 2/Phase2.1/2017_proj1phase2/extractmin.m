function [heap,minvalue,index,indexarray]=extractmin(heap,indexarray)
minvalue=heap(1);
index=indexarray(1);
temp=heap(end);
heap(end)=[];
heap(1)=temp;
temp=indexarray(end);
indexarray(end)=[];
indexarray(1)=temp;

[heap,indexarray]=heapifydown(heap,1,indexarray);
end


