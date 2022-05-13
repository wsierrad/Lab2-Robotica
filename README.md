# Lab2-Robotica
## Integrantes:

Brian Alejandro Vásquez González  
William Arturo Sierra Díaz  

## Mediciones
En primer lugar, se asistió al LABSIR y se tomaron unas series de medidas del robot, y de sus eslabones. A continuación, se hace un resumen de ellas y un diagrama mostrando como se tomaron estas medidas.

-$L_1=47.1 cm$
-$L_2=106.5 cm$
-$L_3=106.5 cm$
-$L_4=69.7 cm$

A continuación un diagrama del robot Phantom X y sus articulaciones.
[![Mediciones.jpg](https://i.postimg.cc/59DLZ8Xz/Mediciones.jpg)](https://postimg.cc/V5BJtrBs)
## Análisis
Con base a las distancias que se tomaron de los eslabones, se establecen los marcos de referencia del robot, y se determinan los parámetros de Denavit-Hartenberg. En la imagen que sigue se muestra la tabla con los parámetros, y el robot con sus respectivos marcos de referencia. 
[![phantomx.jpg](https://i.postimg.cc/rF6RhcHC/phantomx.jpg)](https://postimg.cc/8sHCc8Nj)
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
   
## Toolbox
## Conexión con Matlab
## MATLAB + ROS + Toolbox
