# Obtención del modelo de un robot
## robotmodel.cpp
Codigo que lanza un nodo en ROS2 el cual se encarga de realizar unas trayectorias concretas que se utilizarán posteriormente como modelo del robot. Se envía una velocidad fija y se modifica el ángulo de giro para ir variando las trayectorias en forma de arco que se realizan, al finalizar se almacenan todas las coordenadas y orientaciones que ha experimentado el robot a lo largo de todas esas trayectorias. Se almacenan tanto en un yaml dividido por trayectorias como en ficheros de texto txt.

## modelRL.cpp
Lee los archivos creado por robotmodel.cpp y realiza una regresión polinomial a partir de los puntos para obtener la ecuación de cada curva, una vez obtenidas las ecuaciones realiza una grafica con todas ellas, además las compara con los puntos obtenidos previamente.

## Actualización
Para aumentar la fidelidad del modelo se deberían realizar las mismas trayectorias una y otra vez, cuantos mas puntos se obtengan mas representativo será el modelo posterior.

Una vez obtenido el modelo será necesario programar una función que calcule la siguiente posición del robot dada su posición inicial, una entrada de control y tiempo de ejecución.
