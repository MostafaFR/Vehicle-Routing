function path2root = findpath2root( this, nix )
%%Permit to obtain the index of all the parents nodes to reach the root node
path2root = cat( 2, nix, parent_ixs( nix ) );
    function p2r = parent_ixs( ix )
        if ix == 1
            p2r = [];
        else
            pix = this.Parent( ix );
            p2r = cat( 2, pix, parent_ixs( pix ) );
        end
    end
end