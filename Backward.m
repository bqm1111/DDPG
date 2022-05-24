function [dW] = Backward (nnP,dW_p,O)
   dW = dW_p'*O/size(O,1);
   sign = zeros(size(nnP.W));
   for i=1:length(nnP.W)
       if nnP.W(i) >= 0
           sign(i) = 1;
       else 
           sign(i) = -1;
       end
   end
   dW = sign.*dW;
end