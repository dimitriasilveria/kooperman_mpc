function w = so3_R3(R)
%     sin = 0.5*sqrt((3-trace(R))-(1+trace(R)));
%     log_R = (asin(sin)/2*sin)*(R-R');
%     w1 = log_R(3,2);
%     w2 = log_R(1,3);
%     w3 = log_R(2,1);
%     w = [w1;w2;w3];

    log_R = logm(R);
    w1 = log_R(3,2);
    w2 = log_R(1,3);
    w3 = log_R(2,1);
    w = [w1;w2;w3];

end