# RoboCar-v2

El robot que se muestra a continuación está diseñado para ejecutar los algoritmos que el usuario tenga programados. Para que estos se puedan ejecutar con precisión, se utilizarán una serie de sensores (integrados en una placa de desarrollo) y el Filtro de Kalman.

![alt text](https://github.com/Andresitositoses/RoboCar-v2/tree/main/images/delantera.png)

## Elementos implicados

La plataforma robótica consta de una serie de componentes que se deben interconectar
entre sí para permitir que todo el conjunto funcione correctamente. Los elementos implicados
en este trabajo son los siguientes:

- Soporte de la plataforma (chasis).
- 2 ruedas junto con sus respectivos motores.
- 1 bola giratoria como punto de apoyo.
- Puente H (modelo L298N) para el manejo de las ruedas.
- 2 encoders para medir la velocidad de las ruedas.
- La placa de desarrollo descrita en la sección anterior: B-U585I-IOT02A.
- Una pequeña protoboard para gestionar las conexiones de los encoders hacia la placa.
- Una pila de 9V para la alimentación de las ruedas, junto con un pequeño soporte que nos permite acoplarla.
- Una fuente de alimentación externa para la placa, concretamente una mini Power Bank con 5V de tensión, amperaje de 3A y un peso de 98g.

## Compilación

El núcleo de este robot se trata del microcontrolador STM32U585AI, por lo que se puede gestionar fácilmente mediante el entorno de desarrollo STM32CubeIDE. Desde este, tanto la compilación como el flasheo se pueden realizar de manera automática.

## Utilización de la plataforma robótica

### Calibración inicial

Para el correcto funcionamiento del robot, será necesario pasar por un proceso previo de calibración. Este se completará en dos pasos:
- Calibración de las ruedas, en la que se someterá a las ruedas a diferentes velocidades.
- Calibración de los sensores, donde se deberá rotar manualmente a la plataforma en todos los ejes.

Finalizada la calibración, se encenderá un LED.

### Ejecución del algoritmo

El algoritmo que ejecutará el robot será el especificado en el hilo principal (*mainThread_entry*) del archivo *Src/app_thread.cpp*. En *Src/algorithms.cpp* se encuentran una serie de ejemplos que abarcan toda la funcionalidad de la plataforma robótica.

Establecido el algoritmo y completada la calibración, bastará con pulsar el botón que hay junto al LED mencionado para que el robot comience su ejecución.

### Robot en ejecución

En el siguiente enlace se muestra el proceso de calibración, así como algunos ejemplos: https://youtu.be/xn1RKDEi2k0
