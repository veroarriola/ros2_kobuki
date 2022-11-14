# ros2_kobuki
Paquetes para simular la kobuki

## Prerrequisitos

Instalar los siguientes paquetes, que incluyen los nodos para publicar el estado de las articulaciones y otros auxiliares:
```
$ sudo apt install ros-humble-urdf-tutorial
$ sudo apt install ros-humble-diagnostic-aggregator
```
Para que el análisis de las descripciones de los robots sea correcto, agregar una línea al final del archivo ```.bashrc```:
```
$ echo 'export LC_NUMERIC="es_MX.UTF-8"' >> ~/.bashrc
```

Para utilizar los macros de urdf:
```
$ sudo apt install ros-humble-xacro
```

Crear un espacio de trabajo:
```
$ mkdir -p ~/kobuki_ws/src
```
Dentro de un espacio de trabajo, clonar [kobuki_ros_interfaces](https://github.com/kobuki-base/kobuki_ros_interfaces) dentro de ```src```.

**Terminal 0.1**

Compilar con ```colcon build``` antes de clonar este repo.  Continuar en otra terminal.

## Instalar

Clonar este repo de modo que los directorios queden dentro de ```src``` y compilar con ```colcon```.

**Terminal 0.2**
```
$ cd ~/kobuki_ws/src/
$ git clone <repo> .
$ cd ..
$ colcon build
```

## Ejecutar

### Visualizar el modelo de la Kobuki en Rviz2

**Terminal 1**
```
$ ros2 launch kobuki_description view_model.py
```


### Mover a la kobuki virtual

**Terminal 1**
```
$ ros2 launch kobuki_softnode full.launch.py
```
**Terminal 2**
```
$ ros2 topic pub --once /velocity geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

# Kobuki en Ignition Gazebo

Gazebo trabaja independientemente de ROS 2.  Para indicarle dónde está el modelo se puede usar algo como:

**Terminal 1**
```
$ export IGN_GAZEBO_RESOURCE_PATH="$HOME/kobuki_ws/install/kobuki_description/share"
$ ign gazebo empty.sdf
```
**Terminal 2**
```
$ ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 300 --req 'sdf_filename:"kobuki_description/urdf/kobuki_standalone.urdf" name: "kobuki"'
$ ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.5}"
```
Esto importará a la kobuki dentro un mundo vacío y comenzará a avanzar girando a la izquierda.
