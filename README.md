# ros2_kobuki
Paquetes para simular la kobuki

## Prerrequisitos

```
$ sudo apt install ros-humble-urdf-tutorial
```

Dentro de un espacio de trabajo, clonar [kobuki_ros_interfaces](https://github.com/kobuki-base/kobuki_ros_interfaces) dentro de ```src```.

Compilar con ```colcon build``` antes de clonar este repo.  Continuar en otra terminal.

## Instalar

Clonar este repo de modo que los directorios queden dentro de ```src``` y compilar con ```colcon```.

```
$ git clone <repo> .
```

## Ejecutar

```
$ ros2 launch kobuki_description view_model.py
```
