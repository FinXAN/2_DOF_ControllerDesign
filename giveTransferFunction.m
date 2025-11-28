function T = giveTransferFunction(C1,C2,P)
T = (P*C1)/(1+P*C2);
end