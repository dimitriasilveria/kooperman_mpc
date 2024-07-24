function R = R3_so3(w)
    v3 = w(3,1);
    v2 = w(2,1);
    v1 = w(1,1);
    R = [ 0 , -v3,  v2;
          v3,   0, -v1;
         -v2,  v1,   0];
end