function [D]=distance2(X,Y,A)
% computes sq. A-norm distance between two D-dimensional vectors (rows of X
% and Y)
% X - [Nx x D] matrix
% Y - [Ny x D] matrix
% A - [D x D] matrix (optional input)
if(nargin<3)
    A=eye(size(X,2));
end;

D=bsxfun(@plus,bsxfun(@plus,-2*X*A*Y',sum(X*A.*X,2)),[sum(Y*A.*Y,2)]');