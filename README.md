# Fundamentos de Robótica móvil 

## Requisitos 

* Gazebo
* ROS 

### Instructivo de instalación

Se puede utilizar el [entorno de desarrollo de ROS](https://www.theconstructsim.com/) o  instalar ROS en la máquina con Ubuntu. 

1- Instalar ROS  - [Tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) 

* Agregar a ros.org como fuente de paquetes: 

  `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

* Agregar clave: 

  `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654` 

* Actualizar listado de paquetes: 

  `sudo apt update` 

* Instalar ros-noetic full (con simuladores y dependencias): 

  ` sudo apt install ros-noetic-desktop-full` 

* Agregar el path de ros a la terminal (de forma definitiva). Esto no es necesario 
y se puede hacer solo cada vez que abramos una terminal y queramos los comando de ros  

  `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc` 

* Recargar el bashrc: 
`source ~/.bashrc` 

### Instalar paquetes de TurtleBot3 para la simulación: 

* `mkdir -p ~/catkin_ws/src` 

*  Descargar repositorios para ros-noetic (de su respectiva branch): 
~~~ 
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git ` 
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
 ~~~ 

* Cambiar al directorio padre y construir  escribiendo el comando:  

  `cd ~/catkin_ws && catkin_nmake `


* Agregar al path: 

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc` 

* Recargar el .bashrc: 

`source ~/.bashrc `
 
3- Lanzar la simulación: 

* Elegir el modelo del robot modificando la variable: 

`export TURTLEBOT3_MODEL=burger` 

* Lanzar el nodo - simulación: 

`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch` 

* Enviar comandos de velocidad (*desde una terminal nueva*): 

`rostopic pub -1 /cmd_vel geometry_msgs/Twist '[0.2,0,0]' '[0,0,0.2]'`
