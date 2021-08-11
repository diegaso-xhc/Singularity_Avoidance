function [Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8] = inv_kinematics(x,y,z,xx,xy,xz,yx,yy,yz,zx,zy,zz,UR)

syms  q1 q2 q3 q4 q5 q6 

if UR == 3 
    dU = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921];
    aU = [0, -0.24355, -0.2132, 0, 0, 0];
elseif UR == 10
    dU = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655];
    aU = [0, -0.6127, -0.57155, 0, 0, 0];
%    dU = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
%    aU = [0, -0.612, -0.5723, 0, 0, 0];
end

%% Articulaacion 1
teta = q1;
d = dU(1);
alfa = 90;
a = aU(1);
A01= [[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
A01 = simplify(A01);
R(1).matrix = simplify(A01(1:1:3,1:1:3));
r(1).vector = A01(1:1:3,4);

%% Articulacion 2
teta = q2;
d = dU(2);
alfa = 0;
a = aU(2);
A12= [[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
 A12 = simplify(A12);
 R(2).matrix = simplify(A12(1:1:3,1:1:3)); 
 r(2).vector = A12(1:1:3,4);
 
%% Articulacion 3
teta = q3;
d = dU(3);
alfa = 0;
a = aU(3);
A23 =[[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
 A23 = simplify(A23);
 R(3).matrix = simplify(A23(1:1:3,1:1:3));
 r(3).vector = A23(1:1:3,4);
 
%% Articulacion 4
teta = q4;
d = dU(4);
alfa = 90;
a = aU(4);
A34 =[[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
 A34 = simplify(A34);
 R(4).matrix =simplify( A34(1:1:3,1:1:3));
 r(4).vector = A34(1:1:3,4);
 
 %% Articulacion 5
teta = q5;
d = dU(5);
alfa = -90;
a = aU(5);
A45 =[[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
 R(5).matrix = simplify(A45(1:1:3,1:1:3));
A45 = simplify(A45);
r(5).vector = A45(1:1:3,4);

 %% Articulacion 6
teta = q6;
d = dU(6);
alfa = 0;
a = aU(6);
A56 =[[ cos(teta)  -sin(teta)*cosd(alfa)      sin(teta)*sind(alfa)        a*cos(teta)];...
     [ sin(teta)    cos(teta)*cosd(alfa)     -cos(teta)*sind(alfa)        sin(teta)*a];...
     [ 0                     sind(alfa)                 cosd(alfa)                  d];...
     [          0                      0               0           1]];
R(6).matrix = simplify(A45(1:1:3,1:1:3));
A56 = simplify(A56);
r(6).vector = A56(1:1:3,4);

T = (A01)*(A12)*(A23)*(A34)*(A45)*(A56);




%% Articulation 1
p60 = [x y z];
z60 = [zx zy zz];
p50 = p60 - dU(6).*z60;
phi1 = atan2(p50(:,2),p50(:,1));
phi2 = acos(dU(4)./(sqrt(p50(:,2).^2+p50(:,1).^2)));
q1_1 = unwrap(phi1 + phi2 + pi/2);
q1_2 = unwrap(phi1 - phi2 + pi/2);



%% Articulation 5
p60 = [x y z];
z60 = [zx zy zz];
p50 = p60 - dU(6).*z60;
phi1 = acos((p60(:,1).*sin(q1_1)-p60(:,2).*cos(q1_1)-dU(4))./dU(6));
phi2 = -acos((p60(:,1).*sin(q1_1)-p60(:,2).*cos(q1_1)-dU(4))./dU(6));
phi3 = acos((p60(:,1).*sin(q1_2)-p60(:,2).*cos(q1_2)-dU(4))./dU(6));
phi4 = -acos((p60(:,1).*sin(q1_2)-p60(:,2).*cos(q1_2)-dU(4))./dU(6));
q5_1 = unwrap(phi1);
q5_2 = unwrap(phi2);
q5_3 = unwrap(phi3);
q5_4 = unwrap(phi4);


%% Articulation 6
p60 = [x y z];
z60 = [zx zy zz];
p50 = p60 - dU(6).*z60;
phi1 = atan2((-yx(:,1).*sin(q1_1)+yy(:,1).*cos(q1_1))./sin(q5_1), (xx(:,1).*sin(q1_1) - xy(:,1).*cos(q1_1))./sin(q5_1));
phi2 = atan2((-yx(:,1).*sin(q1_1)+yy(:,1).*cos(q1_1))./sin(q5_2), (xx(:,1).*sin(q1_1) - xy(:,1).*cos(q1_1))./sin(q5_2));
phi3 = atan2((-yx(:,1).*sin(q1_2)+yy(:,1).*cos(q1_2))./sin(q5_3), (xx(:,1).*sin(q1_2) - xy(:,1).*cos(q1_2))./sin(q5_3));
phi4 = atan2((-yx(:,1).*sin(q1_2)+yy(:,1).*cos(q1_2))./sin(q5_4), (xx(:,1).*sin(q1_2) - xy(:,1).*cos(q1_2))./sin(q5_4));
q6_1 = unwrap(phi1);
q6_2 = unwrap(phi2);
q6_3 = unwrap(phi3);
q6_4 = unwrap(phi4);



%% Articulation 3

for i=1:1:length(q1_1)
   q1 = q1_1(i);
   q5 = q5_1(i);
   q6 = q6_1(i);
   T01 = eval(A01);
   T45 = eval(A45);
   T56 = eval(A56);
   T06 = [xx(i),yx(i),zx(i),x(i); ...
       xy(i),yy(i),zy(i),y(i); ...
       xz(i),yz(i),zz(i),z(i); ...
       0 0 0 1 ]; 
   
   T14 = inv(T01)*T06*inv(T56)*inv(T45);
   P4xz = sqrt(T14(1,4)^2+T14(2,4)^2);
   q3_1(i) = acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q3_2(i) = -acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));

   
   if ~isreal(q3_1(i))
        q3_1(i)=nan;
        q3_2(i)=nan;
        q2_1(i)=nan;
        q2_2(i)=nan;
        q4_1(i)=nan;
        q4_2(i)=nan;
   else
        q2_1(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_1(i))/P4xz);
        q2_2(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_2(i))/P4xz);    
        q3 = q3_1(i);
        q2 = q2_1(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_1(i) = atan2(T34(2,1),T34(1,1));

        q3 = q3_2(i);
        q2 = q2_2(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_2(i) = atan2(T34(2,1),T34(1,1)); 

   end

   q1 = q1_1(i);
   q5 = q5_2(i);
   q6 = q6_2(i);   
   T01 = eval(A01);
   T45 = eval(A45);
   T56 = eval(A56);
   T06 = [xx(i),yx(i),zx(i),x(i); ...
       xy(i),yy(i),zy(i),y(i); ...
       xz(i),yz(i),zz(i),z(i); ...
       0 0 0 1 ]; 
   T14 = inv(T01)*T06*inv(T56)*inv(T45);
   P4xz = sqrt(T14(1,4)^2 + T14(2,4)^2);
   q3_3(i) = acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q3_4(i) = -acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   
   if ~isreal(q3_3(i))
        q3_3(i)=nan;
        q3_4(i)=nan;
        q2_3(i)=nan;
        q2_4(i)=nan;
        q4_3(i)=nan;
        q4_4(i)=nan;
   else
        q2_3(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_3(i))/P4xz);
        q2_4(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_4(i))/P4xz);    
        
        q3 = q3_3(i);
        q2 = q2_3(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_3(i) = atan2(T34(2,1),T34(1,1));

        q3 = q3_4(i);
        q2 = q2_4(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_4(i) = atan2(T34(2,1),T34(1,1)); 

   end
   

   q1 = q1_2(i);
   q5 = q5_3(i);
   q6 = q6_3(i);
   T01 = eval(A01);
   T45 = eval(A45);
   T56 = eval(A56);
   T06 = [xx(i),yx(i),zx(i),x(i); ...
       xy(i),yy(i),zy(i),y(i); ...
       xz(i),yz(i),zz(i),z(i); ...
       0 0 0 1 ]; 
   T14 = inv(T01)*T06*inv(T56)*inv(T45);
   P4xz = sqrt(T14(1,4)^2+T14(2,4)^2);
   q3_5(i) = acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q3_6(i) = -acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q2_5(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_5(i))/P4xz);
   q2_6(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_6(i))/P4xz);
   

   if ~isreal(q3_5(i))
        q3_5(i)=nan;
        q3_6(i)=nan;
        q2_5(i)=nan;
        q2_6(i)=nan;
        q4_5(i)=nan;
        q4_6(i)=nan;
   else
        q2_5(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_5(i))/P4xz);
        q2_6(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_6(i))/P4xz);    
        q3 = q3_5(i);
        q2 = q2_5(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_5(i) = atan2(T34(2,1),T34(1,1));

        q3 = q3_6(i);
        q2 = q2_6(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_6(i) = atan2(T34(2,1),T34(1,1)); 

   end
   
   
   q1 = q1_2(i);
   q5 = q5_4(i);
   q6 = q6_4(i);
   T01 = eval(A01);
   T45 = eval(A45);
   T56 = eval(A56);
   T06 = [xx(i),yx(i),zx(i),x(i); ...
       xy(i),yy(i),zy(i),y(i); ...
       xz(i),yz(i),zz(i),z(i); ...
       0 0 0 1 ];
    T14 = inv(T01)*T06*inv(T56)*inv(T45);
   P4xz = sqrt(T14(1,4)^2+T14(2,4)^2);
   q3_7(i) = acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q3_8(i) = -acos((P4xz^2-aU(2)^2-aU(3)^2)/(2*aU(2)*aU(3)));
   q2_7(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_7(i))/P4xz);
   q2_8(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_8(i))/P4xz);
   
   if ~isreal(q3_7(i))
        q3_7(i)=nan;
        q3_8(i)=nan;
        q2_7(i)=nan;
        q2_8(i)=nan;
        q4_7(i)=nan;
        q4_8(i)=nan;
   else
        q2_7(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_7(i))/P4xz);
        q2_8(i) = atan2(-T14(2,4),-T14(1,4)) - asin(-aU(3)*sin(q3_8(i))/P4xz);   
        
        q3 = q3_7(i);
        q2 = q2_7(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_7(i) = atan2(T34(2,1),T34(1,1));

        q3 = q3_8(i);
        q2 = q2_8(i); 
        T12 = eval(A12);
        T23 = eval(A23);
        T34 = inv(T23)*inv(T12)*inv(T01)*T06*inv(T56)*inv(T45);
        q4_8(i) = atan2(T34(2,1),T34(1,1)); 

   end
end

Q1 = [q1_1 q2_1.' q3_1.' q4_1.' q5_1 q6_1];
Q2 = [q1_1 q2_2.' q3_2.' q4_2.' q5_1 q6_1];
Q3 = [q1_1 q2_3.' q3_3.' q4_3.' q5_2 q6_2];
Q4 = [q1_1 q2_4.' q3_4.' q4_4.' q5_2 q6_2];
Q5 = [q1_2 q2_5.' q3_5.' q4_5.' q5_3 q6_3];
Q6 = [q1_2 q2_6.' q3_6.' q4_6.' q5_3 q6_3];
Q7 = [q1_2 q2_7.' q3_7.' q4_7.' q5_4 q6_4];
Q8 = [q1_2 q2_8.' q3_8.' q4_8.' q5_4 q6_4];

end

