function element_matrix=eltono(Xnode, Ynode, Znode, xyres, zres)
element_matrix=[Xnode       Ynode       Znode;
                Xnode+xyres Ynode       Znode;
                Xnode+xyres Ynode+xyres Znode;
                Xnode       Ynode+xyres Znode;
                Xnode       Ynode       Znode+zres;
                Xnode+xyres Ynode       Znode+zres;
                Xnode+xyres Ynode+xyres Znode+zres;
                Xnode       Ynode+xyres Znode+zres;];
end

            
