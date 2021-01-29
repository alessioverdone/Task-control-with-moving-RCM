function J_w = weighted(J)

W = 5*eye(size(J,1), size(J,2));
W_inv= inv(W);
J_w = W_inv*J'*inv(J*W_inv*J')


end
