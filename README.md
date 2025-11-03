# ROSCon25_ES: Stonefish Workshop
Repositorio con los paquetes (**ROS 2**) correspondientes al workshop en [Stonefish](https://github.com/patrykcieslak/stonefish) para la ROSCON25 llevada a cabo en Barcelona, España.

<div align="center">
  <img src="/media/rosconmarine.png" width="500">
  <p>
  <a href="https://roscon.org.es/ROSConES2025.html">ROSCON25_ES</a> / 
  <a href="https://github.com/ros-maritime">ros-maritime-group</a> / 
  <a href="https://stonefish.readthedocs.io/en/latest/">Stonefish</a>  
  </p>
</div>

Tabla de contenido
- [ROSCon25\_ES: Stonefish Workshop](#roscon25_es-stonefish-workshop)
  - [Resumen](#resumen)
  - [Instrucciones uso con Docker](#instrucciones-uso-con-docker)
    - [Instalacion previa](#instalacion-previa)
    - [Compilar y ejecutar el container](#compilar-y-ejecutar-el-container)
  - [Instrucciones uso workspace local](#instrucciones-uso-workspace-local)
    - [Libreria Stonefish](#libreria-stonefish)
    - [Paquetes de ROS](#paquetes-de-ros)

<table><thead>
  <tr>
    <td rowspan="2" align="center"><strong>Responsables Workshop</td>
    <td>Sebastian Realpe Rua</td>
    <td>
    <a href="https://es.linkedin.com/in/sebastian-realpe-rua">linkedin.com/in/sebastian-realpe-rua</a>
    </td>
  </tr>
  <tr>
    <td>Salvador Barajas Lopez</td>
    <td>
    <a href="https://es.linkedin.com/in/salvador-l%C3%B3pez-barajas-3a070a188">linkedin.com/in/salvador-lopez-barajas</a>
    </td>
  </tr></thead>
</table>

## Resumen
En este workshop buscamos introducir [Stonefish](https://github.com/patrykcieslak/stonefish), simulador para robotica marina, el cual puede ser integrado en el ecosistema de ROS con su paquete [stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2).

> Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France.

## Instrucciones uso con Docker
Ofrecemos un entorno contenerizado (**Docker**) con los recursos necesarios para participar de forma activa en el workshop.

Para poder usar el entorno se debe hacer lo siguiente:
### Instalacion previa
1. **Entornos Docker**: Debes tener instalado Docker engine en tu ordenador para poder ejecutar el container: [Install Docker engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

2. **Nvidia container Toolkit**: Nvidia dispone de un toolkit espefico para que entornos contenerizados puedan acceder al hardware grafico del host, por lo que es necesarios instalar el toolkit en tu ordenador: [Install Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)

3. **Clonar este repositorio** Ya estas listo, puedes clonar este repo para ejecutar el Dockerfile asociado y utilizar los paquetes necesarios para el workshop.
```
git clone https://github.com/GitSRealpe/roscon25_es_stonefish_workshop.git
```

### Compilar y ejecutar el container
Procedemos ahora a ejecutar el container, para esto realizamos:
   1. `cd roscon25_es_stonefish_workshop`
   2. `docker compose build` : compilamos el docker el cual usa una imagen precompilada de Stonefish bajo una imagen con ROS 2 Jazzy.
   3. `./run.sh` : Ejecutamos este shell script como utilidad para iniciar el container
      1. `ros2 launch bluerov_stonefish bluerov_sim.launch`
   4. En una pestana nueva de la terminal: `docker exec -it roscon25-stonefish-docker-dev-run-xxxxxx bash` para abrir otra terminal bash en la misma instancia del contenedor.
      1. `ros2 launch roscontrol_test testcontroller.launch.py` para lanzar el stack de ros_control del robot.
   5. Nuevamente, en otra pestana nueva de la terminal: `docker exec -it roscon25-stonefish-docker-dev-run-xxxxxx bash` para una terminal adicional
      1. `ros2 launch bluerov_stonefish slides_deck.launch` para lanzar las slides interactivas del workshop


## Instrucciones uso workspace local
Si tienes listo un workspace de **ROS 2** en tu ordenador, tambien puedes hacer la instalacion manual de los paquetes, para ello sigue estas instrucciones:

### Libreria Stonefish
[Stonefish](https://github.com/patrykcieslak/stonefish) como tal es una libreria desarrollada en C++, independiente de ROS, por lo cual debe ser instalada como cualquier libreria.

1. Instalar dependencias [OpenGL Mathematics library, SDL2 library, Freetype library]:
```
sudo apt install libglm-dev libsdl2-dev libfreetype6-dev
```
2. Compilar e instalar
   1. Crear directorio para compilar `mkdir ~/libs` y `cd ~/libs`
   2. Clonar [Stonefish](https://github.com/patrykcieslak/stonefish) repo.
   3. `cd stonefish`
   4. `mkdir build`
   5. `cd build`
   6. `cmake ..`
   7. `make -jX` (X es el numero de procesos a usar)
   8. Instalar libreria en tu ordenador `sudo make install`

### Paquetes de ROS
Luego de esto procedemos a clonar y compilar el paquete que enlaza la libreria Stonefish con ROS 2.