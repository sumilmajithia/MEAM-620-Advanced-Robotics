function [G,indexarray,nodeindexarray]=heapifydown(G,index,indexarray,nodeindexarray)
while index <=numel(G)
    smallest=index;
    child1= index*2;
    child2=index*2+1;
    if child1 <= numel(G) && G(child1)< G(smallest)
        smallest=child1;
    end
      if child2 <= numel(G) && G(child2)< G(smallest)
        smallest=child2;
      end
      if smallest ~= index
          temp=G(index);
          G(index)=G(smallest);
          G(smallest)=temp;
          temp=indexarray(index);
          indexarray(index)=indexarray(smallest);
          indexarray(smallest)=temp;
          nodeindexarray(indexarray(index))=index;
          nodeindexarray(indexarray(smallest))=smallest;
          index=smallest;
      else 
          break;
      end
end
end