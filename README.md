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
<!-- INSTRUCCIONES INSTALAR DOCKER Y COSAS -->
## Instrucciones uso workspace local
Si tienes listo un workspace de ROS 2 en tu ordenador, tambien puedes hacer la instalacion manual de los paquetes, para ello sigue estas instrucciones:

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