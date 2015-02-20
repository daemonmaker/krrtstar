function [Uout,Sout,Vout] = SVD(A)
% given an arbitrary mxn matrix (m >= n), computes its svd
% and returns it in U,S,V, where A = U*S*V, U,V unitary, and S diagonal,
% where s_i >= 0, and s_i >= s_i+1
% if called with less than three output arguments, will return only S
% eps is used for the tolerance, but is ignored.
% this svd algo is slightly inefficient, i.e. spread over a couple of
% functions that call each other a lot (we use the Kogbetliantz method)
  m = size(A,1); % get the size of the matrix
  n = size(A,2);
  U = eye(m);
  V = eye(n);
  e = eps*fro(A);
  while (sum(abs(A(~eye(m,n)))) > e) % termination condition
      for i = 1:n
          for j = i+1:n
              [J1,J2] = jacobi(A,m,n,i,j);
              A = mtimes(J1,mtimes(A,J2));
              U = mtimes(U,J1');
              V = mtimes(J2',V);
          end
          for j = n+1:m
              J1 = jacobi2(A,m,n,i,j);
              A = mtimes(J1,A);
              U = mtimes(U,J1');
          end
      end
  end
  S = A;
  % check if we need less than three output arguments
  if (nargout < 3)
      Uout = diag(S);
  else
      Uout = U; Sout = times(S,eye(m,n)); Vout = V;
  end
end


% finds the jacobi rotation that will zero A(i,j), A(j,i), A a mxn matrix
function [J1,J2] = jacobi(A,m,n,i,j)
   B = [A(i,i), A(i,j); A(j,i), A(j,j)]; % get little matrix out of A
   [U,S,V] = tinySVD(B); % get its svd
   
   J1 = eye(m);
   J1(i,i) = U(1,1);
   J1(j,j) = U(2,2);
   J1(i,j) = U(2,1);
   J1(j,i) = U(1,2);
   
   J2 = eye(n);
   J2(i,i) = V(1,1);
   J2(j,j) = V(2,2);
   J2(i,j) = V(2,1);
   J2(j,i) = V(1,2);
end


% finds the jacobi rotation that will zero A(j,i), A a mxn matrix
function J1 = jacobi2(A,m,n,i,j)
   B = [A(i,i), 0; A(j,i), 0]; % get little matrix out of A
   [U,S,V] = tinySVD(B); % get its svd
   
   J1 = eye(m);
   J1(i,i) = U(1,1);
   J1(j,j) = U(2,2);
   J1(i,j) = U(2,1);
   J1(j,i) = U(1,2);
end



% given an arbitrary 2x2 matrix, computes its svd
% and returns it in U,S,V, where A = U*S*V, U,V unitary, and S diagonal,
% where s_i >= 0, and s_i >= s_i+1
% if called with less than three output arguments, will return only S
function [Uout,Sout,Vout] = tinySVD(A)
  t = rdivide((minus(A(1,2),A(2,1))),(plus(A(1,1),A(2,2))));
  c = rdivide(1,sqrt(1+t^2));
  s = times(t,c);
  R = [c,-s;s,c];  
  M = mtimes(R,A); % find symmetric matrix
  [U,S,V] = tinySymmetricSVD(M);
  U = mtimes(R',U);

  % check if we need less than three output arguments
  if (nargout < 3)
      Uout = diag(S);
  else
      Uout = U; Sout = S; Vout = V;
  end
end



% given a symmetric 2x2 matrix, computes its svd
% and returns it in U,S,V, where A = U*S*V, U,V unitary, and S diagonal,
% where s_i >= 0, and s_i >= s_i+1
% if called with less than three output arguments, will return only S
function [Uout,Sout,Vout] = tinySymmetricSVD(A)
  if (A(2,1) == 0) % case where it's already symmetric
     S = A;
     U = eye(2);
     V = U;
  else % case where off diagonals are not 0
     % taken directly from the notes
     w = A(1,1);
     y = A(2,1);
     z = A(2,2);
     ro = rdivide(minus(z,w),times(2,y));
     t2 = rdivide(sign(ro),plus(abs(ro),sqrt(plus(times(ro,ro),1))));
     t = t2;
     c = rdivide(1,sqrt(plus(1,times(t,t))));
     s = times(t,c);
     U = [c, -s; s, c];
     V = [c,  s;-s, c];
     S = mtimes(U,mtimes(A,V));
     U = U';
     V = V';
  end
  % make sure everything is descending etc...
  [U,S,V] = fixSVD(U,S,V);
 
  % check if we need less than three output arguments
  if (nargout < 3) 
      Uout = diag(S);
  else 
     Uout = U; Sout = S; Vout = V;
  end
end


% takes matrizes U,S,V and returns matrizes U,S,V s.t. S is positive, and
% ordered in descending order
% this only works for 2x2 matrizes
function [U,S,V] = fixSVD(U,S,V)
  Z = [sign(S(1,1)),0; 0,sign(S(2,2))]; % the diagonal matrix holding the signs of elts in S
  U = mtimes(U,Z);
  S = mtimes(Z,S);
  if (S(1,1) < S(2,2))
     P = [0,1;1,0];
     U = mtimes(U,P);
     S = mtimes(P,mtimes(S,P));
     V = mtimes(P,V);
  end
end


% calculates the frobenius norm
function f = fro(M)
  f = sqrt(sum(sum(times(M,M))));
end


% we will override sign, so that sign(0) = 1
function s = sign(x) 
   if (x > 0) 
       s = 1;
   else
       s = -1;
   end
end
