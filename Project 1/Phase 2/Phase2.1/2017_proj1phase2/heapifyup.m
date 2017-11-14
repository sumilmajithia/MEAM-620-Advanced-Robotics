function [G,indexarray] =heapifyup(G,index,indexarray)
while index~=1

if mod(index,2)==1
    parentindex=(index-1)/2;
else
    parentindex=index/2;
end
if G(index) < G(parentindex)
    temp=G(parentindex);    
    G(parentindex)=G(index);
    G(index)=temp;
    temp=indexarray(parentindex);    
    indexarray(parentindex)=indexarray(index);
    indexarray(index)=temp;
    
end
index=parentindex;
end
end