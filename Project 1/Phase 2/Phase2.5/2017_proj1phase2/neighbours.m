function A=neighbours(index,lx,ly,lz)

xresidue=mod(index,lx);
yresidue=mod(index,lx*ly);
zresidue=mod(index,lx*ly*lz);
A=[];
if xresidue==0
    A=[A,index-1];
else 
    if xresidue==1
        A=[A,index+1];
    else
        A=[A,index+1];
        A=[A,index-1];
    end
end
if yresidue==0
    A=[A,index-lx];
else 
    if yresidue==1
        A=[A,index+lx];
    else
        A=[A,index+lx];
        A=[A,index-lx];
    end
end
if zresidue==0
    A=[A,index-lx*ly];
else 
    if zresidue==1
        A=[A,index+lx*ly];
    else
        A=[A,index+lx*ly];
        A=[A,index-lx*ly];
    end
end
 A=A(A>0);
 A=A( A<=lx*ly*lz);
end