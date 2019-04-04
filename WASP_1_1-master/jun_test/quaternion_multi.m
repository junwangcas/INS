
w_tb= [30*pi/180;0;0];
dt = 1;
q_input = [0;0;0;1];


P=w_tb(1)*dt;
Q=w_tb(2)*dt;
R=w_tb(3)*dt;


OMEGA=zeros(4);
OMEGA(1,1:4)=0.5*[0 R -Q P];
OMEGA(2,1:4)=0.5*[-R 0 P Q];
OMEGA(3,1:4)=0.5*[Q -P 0 R];
OMEGA(4,1:4)=0.5*[-P -Q -R 0];

v=norm(w_tb)*dt;

if v~=0
    disp('quaternion:');
    q_output=(cos(v/2)*eye(4)+2/v*sin(v/2)*OMEGA )*q_input
end
