%% Conexión con Matlab:
%Cree un script que permita publicar en cada topico de controlador de junta, se deben validar los limites articulares de cada junta.
%Cree un script que permita suscribirse a cada topico de controlador de junta, el script debe retornar la configuracion de 5 angulos en radianes.
l = [4.7, 10.65, 10.65, 6.97]; % Longitudes eslabones
% Definicion del robot RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
% roty(pi/2)*rotz(-pi/2)
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
ws = [-50 50];
% Graficar robot
pos= [0 0 0 0];
q = deg2rad(pos);
PhantomX.plot(q,'notiles','noname');
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])
PhantomX.teach()
PhantomX.fkine(q);
% Ahora, las MTH
syms q1 q2 q3 q4 
d1=l(1);
ap1=pi/2;
T01= simplify(trotz(q1)*transl(0,0,d1)*trotx(ap1));
 T01(1,2) = 0;
 T01(2,2) = 0;
 T01(3,3) = 0
a2=l(2);
T12=simplify(trotz(q2)*transl(a2,0,0))
a3=l(3);
T23=simplify(trotz(q3)*transl(a3,0,0))
a4=l(4);
T34=simplify(trotz(q4)*transl(a4,0,0))
T0T=simplify(T01*T12*T23*T34*PhantomX.tool)
PhantomX.teach();
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])
figure 
PhantomX.teach(q)


% Conexión con nodo maestro
% Inicia la conexión con el nodo maestro por default en localhost por el puerto 11311. 
rosinit;

% Configuracion de publicador
% Creación del publicador, se define el nombre del topico y el tipo de mensaje
motorSVC = rossvcclient('dynamixel_workbench/dynamixel_command');
% Creación de mensaje para su publicación
velMsg = rosmessage(motorSVC); 
velMsg.AddrName = "Goal_Position";
for i = 1:length(pos) 
    velMsg.Id = i;
    value = round(mapfun(pos(i),-150,150,0,1023))
    velMsg.Value = value;
    call(motorSVC,velMsg);
    pause(1);
end

% ROSsuscriber
% Creación del suscriptor, se define el nombre del topico y el tipo de mensaje
poseSub = rossubscriber("dynamixel_workbench/joint_states","sensor_msgs/JointState");
% Pausa de 1ms mientras se recibe el primer mensaje
pause(1)
% Se toma el ultimo mensaje publicado por el topico
scanMsg = poseSub.LatestMessage
position = scanMsg.Position

%Finalizar la conexion al nodo maestro de ROS
rosshutdown;

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
