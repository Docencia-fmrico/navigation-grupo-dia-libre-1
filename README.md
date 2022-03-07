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

![bt](https://user-images.githubusercontent.com/78978326/157094597-1a286c4f-3bc7-485a-baf6-cd9510f01947.jpeg)



Sequence:

  -GetNextWaypoint:   Este nodo va pasando puntos al Move, cada vez que se pasa por aqui se avanza una posición en la lista de puntos, siempre tiene exito. Output = "waypoint"

  -Move:   Coge el punto que se le pasa y va hacia él, devuelve éxito y emite un sonido si se consigue llegar, y devuelve fallo y otro sonido y no consigue llegar al punto. Input  = "waypoint"
  
  -IsRaceFinished:   Este nodo comprueba si el punto al que ha ido el Move es el mismo que el último al que hay que ir. Fallo si no lo es, éxito si era el último, por lo tanto el Sequence devuelve éxito y se termina.
  
En cuando al bt_navigator hemos usado el navigate_w_replanning_and_recovery.xml.

Con respecto a los controllers servers hicimos distintas pruebas para elegir con que controlador trabajariamos:

Primero probamos la navegación con el controller default, el cual realizaba las rutas de forma correcta y no tardaba demasiado en pensar.

A continuación hicimos varias pruebas con el controlador pursuit el cual parecía que navegaba de manera más rápida y fluida pero tenía un
inconveniente y es que en muchas situaciones el robot se quedaba mucho rato pensando (por ejemplo cuando alcanzaba un waypoint) esto generaba que a corto medio plazo, el algoritmo fuera más lento y menos eficiente. de esa manera concluimos que para la tarea que habrán que realizar en clase la cuál consiste en alcanzar varios waypoints seguidos, el controlador inicial es el mas adecuado.

Después de probar con el controlador pursuit quizimos también evaluar el comportamiento del controlador shim sin embargo como varios de nuestros compañeros no conseguimos hacer que este funcionara correctamente

Con todo esto en cuenta decidimos que para la tarea encomendada el controlador que nos ofrecerá un mejor comportamiento era el inicial.

