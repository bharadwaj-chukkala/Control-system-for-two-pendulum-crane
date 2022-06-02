syms M m1 m2 l1 l2 g;
% Creating a linearised state space equation using A and B matrices
A=[0 1 0 0 0 0; 
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];         %Initializing the B matrix
C1 = [1 0 0 0 0 0];                           %Corresponding to x component
C2 = [0 0 1 0 0 0; 0 0 0 0 1 0];              %corresponding to theta1 and theta2
C3 = [1 0 0 0 0 0; 0 0 0 0 1 0];              %cooresponding to x and theat2
C4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; %cooresponding to x, theta1 and theat2
%Matrix to check th Observability Condition
Observability1 = [C1' A'*C1' A'*A'*C1' A'*A'*A'*C1' A'*A'*A'*A'*C1' A'*A'*A'*A'*A'*C1'];
Observability2 = [C2' A'*C2' A'*A'*C2' A'*A'*A'*C2' A'*A'*A'*A'*C2' A'*A'*A'*A'*A'*C2'];
Observability3 = [C3' A'*C3' A'*A'*C3' A'*A'*A'*C3' A'*A'*A'*A'*C3' A'*A'*A'*A'*A'*C3'];
Observability4 = [C4' A'*C4' A'*A'*C4' A'*A'*A'*C4' A'*A'*A'*A'*C4' A'*A'*A'*A'*A'*C4'];
rankArray = [rank(Observability1),rank(Observability2),rank(Observability3),rank(Observability4)];

% Iterating Over rankArray
for i = 1:4
  switch(i)
      case 1
          if rankArray(i)==6 %Checking if rank is 6
              disp('System is observable, when only x(t) is requested!')
            else
              disp('System is not observable, when only x(t) is requested!')
          end
      case 2
           if rankArray(i)==6 %Checking if rank is 6
              disp('System is observable, when only theta1(t) and theta2(t) is requested!')
           else
              disp('System is not observable, when only theta1(t) and theta2(t) is requested!')
           end
      case 3
           if rankArray(i)==6 %Checking if rank is 6
                   disp('System is observable, when only x(t) and theta2(t) is requested!')
            else
                disp('System is not observable, when x(t), theta1(t) and theta2(t) is requested!')
           end

      case 4
           if rankArray(i)==6 %Checking if rank is 6
                disp('System is observable, when x(t), theta1(t) and theta2(t) is requested!')
            else
                disp('System is not observable, when x(t), theta1(t) and theta2(t) is requested!')
            end
      
  end
end