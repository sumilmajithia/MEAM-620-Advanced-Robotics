function C=skipoints(map,p1,p2)
points=zeros(1000,3);

for i=1:1000
    points(i,1)=p2(1) +(p1(1)-p2(1))*(i-1)*0.001;
    points(i,2)=p2(2) +(p1(2)-p2(2))*(i-1)*0.001;
    points(i,3)=p2(3) +(p1(3)-p2(3))*(i-1)*0.001;
end
    

 c=collide1(map,points);
 
 if c==zeros(1000,1)
     C=1;
 else
     C=0;
 end
end