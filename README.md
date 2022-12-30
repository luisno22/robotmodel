# Obtención del modelo de un robot
## robotmodel.cpp
Codigo que lanza un nodo en ROS2 el cual se encarga de realizar unas trayectorias concretas que se utilizarán posteriormente como modelo del robot. Se envía una velocidad fija y se modifica el ángulo de giro para ir variando las trayectorias en forma de arco que se realizan, al finalizar se almacenan todas las coordenadas y orientaciones que ha experimentado el robot a lo largo de todas esas trayectorias. Se almacenan tanto en un yaml dividido por trayectorias como en ficheros de texto txt.

## modelRL.cpp
Lee los archivos creado por robotmodel.cpp y realiza una regresión polinomial a partir de los puntos para obtener la ecuación de cada curva, devolviendo así los coeficientes de la ecuación que define cada curva.

## plot.py
Realiza diversas gráficas. Gráfica de los valores reales obtenidos en el modelo sin ningun tipo de procesamiento (Figure_1.png). Gráfica por curvas donde se puede apreciar cada curva de un color distinto (Figure_2.png). Gráfica de las curvas obtenidas utilizando las ecuaciones obtenidas en modelRL.cpp (por ahora deben ser introducidos a mano los coeficientes de dichas ecuaciones, Figure_3.png)

## Actualización
Para aumentar la fidelidad del modelo se deberían realizar las mismas trayectorias una y otra vez, cuantos mas puntos se obtengan mas representativo será el modelo posterior.

Una vez obtenido el modelo será necesario programar una función que calcule la siguiente posición del robot dada su posición inicial, una entrada de control y tiempo de ejecución.

Obtenida dicha función este modelo será utilizado para programar un control por aprendizaje por refuerzo.


## Especificaciones
Modelo realizado en una nave industrial utilizando para ello una silla de ruedas motorizada llamada Infinity X modificada para poder controlar sus motores desde software.
