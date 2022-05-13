# Lab2-Robotica
## Integrantes:

Brian Alejandro Vásquez González  
William Arturo Sierra Díaz  

## Mediciones
En primer lugar, se asistió al LABSIR y se tomaron unas series de medidas del robot, y de sus eslabones. A continuación, se hace un resumen de ellas y un diagrama mostrando como se tomaron estas medidas.

- L_1 =4.71 cm

- L_2=10.65 cm

- L_3=10.65 cm

- L_4=6.97 cm

A continuación un diagrama del robot Phantom X y sus articulaciones.

[![Mediciones.jpg](https://i.postimg.cc/59DLZ8Xz/Mediciones.jpg)](https://postimg.cc/V5BJtrBs)

## Análisis
Con base a las distancias que se tomaron de los eslabones, se establecen los marcos de referencia del robot, y se determinan los parámetros de Denavit-Hartenberg. En la imagen que sigue se muestra la tabla con los parámetros, y el robot con sus respectivos marcos de referencia. 
[![phantomx.jpg](https://i.postimg.cc/NjTk1z9J/phantomx.jpg)](https://postimg.cc/YvrF78xY)
## ROS
Lo siguiente indicado en la guía, es crear un script de Python que permita controlar cada una de las articulaciones del robot, utilizando los tópicos y servicios de ROS necesarios para este fin. En primer lugar, se crea un script llamado `movePXRobot.py`.  En el se importan inicialmente el cliente de Python para ROS, el módulo `termios` para recibr la entrada por teclado, y los ,emsajes de Dynamixel, para enviar comandos a los motores que controlan cada valor de articulación.

Lo siguiente, es definir las posiciones objetivo que recibirá el programa: la posición de home deseada para el robot, y la posición objetivo que se quiere que siga cada articulación. Se asignan los nombres para cada uno de estas articulaciones, segun la convención antropomórfica: waist, shoulder, elbow y wrist. También se asigna un `id` , que corresponde a la articulación actual, y una variable  `change` que funciona como un flag, para que se haga el cambio de articulación sin alterar su valor.
```python
home_pos = 512
obj_pos_waist = 820
obj_pos_shoulder = 650
obj_pos_elbow = 300
obj_pos_wrist = 204
pos_obj = [obj_pos_waist, obj_pos_shoulder, obj_pos_elbow, obj_pos_wrist]
name = ['waist','shoulder','elbow','wrist']
id = 1
change = 0
```
Se define una función jointCommand, tomada de uno de los scripts compartidos por el profesor, en el que se inicia un nodo, y se llama al servicio `dynamixel_command` para controlar cada articulación. 

```python
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
```
Se hace uso del código brindado para la detección de las teclas oprimidas, y se inicia con el `main`. Se establece la posición inicial, y se indican los limites de torque para cada articulación. 


 ```python
if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        # map(0,1023,-150,150)
        pos = home_pos
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
   ```
Luego, se obtiene la tecla oprimida, y se realiza una determinada acción por cada una de ellas.

-Para la w, si se encuentra en una articulación diferente a la última, se pasa a la siguiente articulación.

-Para la s, si se encuentra en una articulación diferente a la primer, se pasa a la articulación anterior.

-Para la d, se asigna el valor de la posición objetivo, y se pone la variable `change` en 1, para indicar que el valor de la articulación ha cambiado.

-Para la a, se asigna el valor de la posición de home en  la articulación en su posición de home, y se pone la variable `change` en 1.

 ```python
        while(1):
            key = getkey()
            if key == b'w':
                if id != 4:
                    id =  id + 1
                else:
                    id = 1
            if key == b's':
                if id != 1:
                    id = id - 1
                else:
                    id = 4
            if key == b'd':
                pos = pos_obj[id-1]
                change = 1
            if key == b'a':
                pos = home_pos
                change = 1
```
Finalmente, como comprobación, se imprimen los valores de la junta actual, y su valor actual de posición, además si la variable `change` está en 1, se ejecuta el comando de movimient de la articulación, y se devuelve la `change` a 0, para evitar que  al moverse a otra articulación, esta se mueva inmediatamente.
```python
            print('Junta: ' + name[id-1] + str(id))
            print('Posicion: ' + str(pos))
            if change == 1:
                jointCommand('', id, 'Goal_Position',pos, 0.5)
                time.sleep(0.5)
                change = 0
    except rospy.ROSInterruptException:
        pass
   ```
Finalmente podemos observar el resultado de la configuracion dada en el visualizador RVIZ
[![RVIZ.png](https://i.postimg.cc/fbphJgK6/RVIZ.png)](https://postimg.cc/5HBrrgD3)
A continuacion se puede visualizar en YouTube el video de los resultados obtenidos:

[Ver video ROS, Python y RVIZ](https://youtu.be/2MRzVuKd-Z4)
## Toolbox
Lo siguiente es la construcción del robot utilizando el comando SerialLink. Para eso se deben especificar las distancias, y los parámetors DH antes especificados. Igualmente, se crea la matriz de la herramienta con respecto al último marco de referencia. 
```matlab
L_1(1) = Link('revolute','alpha',pi/2,'a',0,'d',l(1),'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
L_1(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,'offset',pi/2, 'qlim',[-3*pi/4 3*pi/4]);
L_1(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
L_1(4) = Link('revolute','alpha',0,'a',l(4),'d',0,'offset',0, 'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L_1,'name','Px');
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
```
Lo siguiente es realizar el cálculo de la cinemática directa, y obtener la matriz que relaciona la base con la herramienta, más específicamente con el TCP. 
Se calculan todas las MTH, y al final se obtiene la siguiente matriz:

[![T0T.jpg](https://i.postimg.cc/kMfMfM3z/T0T.jpg)](https://postimg.cc/5jQMtVtS)

Lo siguiente es mostrar el robot en distintas configuraciones. Se establece la posición de home, y se grafica en esta configuración. 
```matlab
q0=[0 0 0 0];
PhantomX.plot(q0, 'notiles', 'noname');
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
ws = [-50 50];
axis([repmat(ws,1,2) 0 60])
PhantomX.fkine(q0);
PhantomX.teach(q0)
```
[![Home.jpg](https://i.postimg.cc/bwHqSqTm/Home.jpg)](https://postimg.cc/nXzy6bNm)

Otra configuración:
```matlab
q1=deg2rad ([90 -45 -30 45])
```
[![Conf1.jpg](https://i.postimg.cc/XJhW6qjY/Conf1.jpg)](https://postimg.cc/gXvQvYLC)

Y otra:
```matlab
q2=deg2rad ([45 45 -60 -30])
```
[![Conf2.jpg](https://i.postimg.cc/3WQHWqDM/Conf2.jpg)](https://postimg.cc/2LGtK2wT)

Y por último:
```matlab
q3=deg2rad ([20 -15 -70 10])
```
[![Conf3.jpg](https://i.postimg.cc/NG999rrb/Conf3.jpg)](https://postimg.cc/pyvX3rCn)

## Conexión con Matlab
Tras tener el robot completamente establecido, se continuó con el laboratorio creando 
```python
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
```
## MATLAB + ROS + Toolbox
A continuacion se puede visualizar en YouTube el video de los resultados obtenidos:
[Ver video ROS y MatLab](https://youtu.be/wd5omj4S2GA)

## Conclusiones
+ ROS por medio del uso de los servicios y nodos permite de una forma estructurada el control del robot Phantom X por medio de script tanto en MatLab como en Python
+ RVIZ es un visualizador que nos permite conectar los servicios de ROS y asi podemos tener una vista virtual del robot y sus movimientos en la vida real
+ Por medio del toolbox RVC tools podemos simular las posiciones y orientaciones del robot y luego comparar con los movimientos que este efectua en el espacio fisico
+ La fidelidad de la visualizacion en RVIZ depende en gran medida de la fidelidad de nuestras medidas y modelos CAD realizados
