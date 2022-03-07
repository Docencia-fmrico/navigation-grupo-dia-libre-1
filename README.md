# Práctica de navegación

[![GitHub Action
Status](https://github.com/Docencia-fmrico/navigation/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/navigation)


**Entrega:** Miércoles 2/3 

En la moqueta verde del laboratorio se limitará con unas paredes, y se pondrán obstáculos (cajas) dentro el viernes 25/2. No habrá cambios en el escenario desde este momento. El miércoles, al inicio de la clase se proporcionarán un conjunto de waypoints en un fichero de parámetros como este:

```
patrolling_node:
  ros__parameters:
    waypoints: ["wp1", "wp2"]
    wp1: [1.0, 1.0]
    wp2: [-1.0, -1,0]
```

El robot debe ir en orden la coordenada (x, y) de cada uno de ellos, emitiendo un sonido cuando lo considera alcanzado. Se cronometrará el tiempo que tarda en hacerlo.

La velocidad lineal no podrá ser nunca superior a 0.4. Se descalificará a quien incumpla esta regla.

Habrá dos rondas:

- Ronda 1: Habrá 4 waypoints, y ninguno en la posición de un obstáculo.
- Ronda 2: Habrá 3-7 waypoints, alguno de ellos en la posición de un obstáculo. En este caso, se podrá ir al siguiente en cuanto se detecte este caso.


# Resolución

Para la implementación del Behaviour Tree hemos usado el Sequence como control. Con la idea de que en secuencia ir pasando los puntos al Move y que este se mueva hasta que no queden más puntos.

La idea del Behavior Tree es la siguiente


Sequence:

  -GetNextWaypoint:   Este nodo va pasando puntos al Move, cada vez que se pasa por aqui se avanza una posición en la lista de puntos, siempre tiene exito. Output = "waypoint"

  -Move:   Coge el punto que se le pasa y va hacia él, devuelve éxito y emite un sonido si se consigue llegar, y devuelve fallo y otro sonido y no consigue llegar al punto. Input  = "waypoint"
  
  -IsRaceFinished:   Este nodo comprueba si el punto al que ha ido el Move es el mismo que el último al que hay que ir. Fallo si no lo es, éxito si era el último, por lo tanto el Sequence devuelve éxito y se termina.
  
En cuando al bt_navigator hemos usado el navigate_w_replanning_and_recovery.xml.

Y tras probar varios controller servers nos hemos quedado con el inicial.

Con el Pursuit controller sabiamos que tardaba en procesar pero hemos creido que era demasiado y no era eficiente.

Con el Shim controller directamente no nos iba bien asi que por eso hemos decidido usar el inicial.

