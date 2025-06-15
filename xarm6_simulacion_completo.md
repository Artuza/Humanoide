
# Simulación y Análisis del Robot xArm6 en ROS 2 y Gazebo

**Clarissa Gardea Coronado**, Tecnológico de Monterrey, Zapopan, México — a01569420@tec.mx  
**Arturo Azael Godínez Rodríguez**, Tecnológico de Monterrey, Zapopan, México — a01641179@tec.mx  
**David Gómez Carrillo**, Tecnológico de Monterrey, Zapopan, México — a01642824@tec.mx  
**Andrés Lepe Alvarado**, Tecnológico de Monterrey, Zapopan, México — a01643265@tec.mx  
**Christian Omar Payán Torróntegui**, Tecnológico de Monterrey, Zapopan, México — a01742658@tec.mx  

---

## Resumen

Este trabajo presenta la simulación, modelado y control del brazo robótico *xArm6* utilizando el entorno robótico ROS 2. Se realizó un análisis cinemático mediante el método de Denavit-Hartenberg (DH) para describir la cadena cinemática del manipulador, cuyos resultados fueron validados visualmente en RViz mediante la publicación de un marcador en la posición del efector final. Además, se implementó la simulación del robot en Gazebo con el entorno `xarm_gazebo`, configurando controladores con `ros2_control` para ejecutar trayectorias articuladas. Finalmente, se diseñó e implementó una secuencia de movimientos tipo sentadilla como demostración de control personalizado, validando así la coherencia entre el modelo analítico, el descriptivo (URDF) y el comportamiento simulado.

**Palabras clave**: xArm6, ROS 2, Gazebo, RViz, ROS Control, Cinemática inversa Denavit-Hartenberg, Control de trayectorias, URDF

---

## Introducción

En este trabajo se llevó a cabo la simulación y control del brazo robótico xArm6 utilizando ROS 2. Se trabajó principalmente con los entornos de Gazebo y RViz para visualizar la configuración del robot y validar su funcionamiento. La simulación incluyó la activación de controladores, la ejecución de trayectorias y la representación del entorno con una mesa como obstáculo.

Como parte de la práctica, se configuraron los controladores necesarios mediante comandos de ROS 2 Control y se utilizó un nodo en Python para enviar una trayectoria específica que simula un movimiento de flexión del brazo. Esta trayectoria fue ejecutada correctamente en el simulador Gazebo, permitiendo observar la respuesta del robot ante comandos personalizados. Además, se comprobó la comunicación entre los distintos nodos y tópicos del sistema, asegurando que el robot respondiera adecuadamente a los mensajes de control.

---

## Análisis Cinemático: Tabla de Denavit-Hartenberg

Con el objetivo de analizar el comportamiento cinemático del brazo robótico *xArm6*, se ha utilizado el método de Denavit-Hartenberg (DH) para describir su cadena cinemática. Este método permite expresar cada transformación entre eslabones consecutivos mediante cuatro parámetros: el ángulo articular \(\theta_i\), la distancia \(d_i\), la longitud del eslabón \(a_i\), y el ángulo de torsión \(\alpha_i\).

Los parámetros DH se obtuvieron a partir del modelo URDF oficial del xArm6. Se seleccionaron marcos de referencia que cumplieran con las convenciones clásicas DH, asignando los ejes \(z\) como ejes de rotación y calculando distancias y ángulos relativos.

**Tabla de parámetros DH:**

| Articulación \(i\) | \(\theta_i\) (rad) | \(d_i\) (m) | \(a_i\) (m) | \(\alpha_i\) (rad) |
|---------------------|----------------------|---------------|---------------|------------------------|
| 1                   | \(\theta_1\)       | 0.267         | 0             | \(-\frac{\pi}{2}\)  |
| 2                   | \(\theta_2\)       | 0             | 0.293         | 0                      |
| 3                   | \(\theta_3\)       | 0             | 0.300         | 0                      |
| 4                   | \(\theta_4\)       | 0.302         | 0             | \(-\frac{\pi}{2}\)  |
| 5                   | \(\theta_5\)       | 0             | 0             | \(\frac{\pi}{2}\)   |
| 6                   | \(\theta_6\)       | 0.072         | 0             | 0                      |

---

## Visualización de la Posición del Efector Final en RViz

Se eligió una configuración articular:

\[
\theta = [0^\circ, 0^\circ, 90^\circ, 0^\circ, 0^\circ, 0^\circ]
\]

Las matrices homogéneas se calcularon como:

\[
A_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

Y la transformación total:

\[
T_0^6 = A_1 A_2 A_3 A_4 A_5 A_6
\]

El vector de posición del efector se extrajo de la última columna de \(T_0^6\) y se comparó visualmente en RViz con el marcador generado por `tf2`.

---

## Simulación en Gazebo con controladores

El entorno simulado se inició con:

```bash
ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py
```

Activación de controladores:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active xarm6_traj_controller
```

Verificación del tópico:

```bash
ros2 topic list | grep trajectory
```

Los movimientos se enviaron con `FollowJointTrajectory` desde un script en Python. La simulación mostró un comportamiento coherente con el modelo y sin colisiones.

---

## Generación heurística de una sentadilla

La sentadilla fue implementada como una secuencia de configuraciones articulares:

1. **Postura inicial**  
2. **Descenso parcial**  
3. **Descenso completo**  
4. **Regreso a la postura inicial**

Se evitó colisión con la mesa y se validó visualmente en Gazebo. La transición entre poses fue suave y precisa.

---

## Conclusión

Se integraron herramientas de ROS 2 para modelar, simular y controlar el robot xArm6. La tabla DH fue validada con RViz y se ejecutaron trayectorias articuladas en Gazebo. El ejercicio de la sentadilla permitió probar la coherencia entre el modelo matemático y el simulado. Este trabajo abre la posibilidad de implementar algoritmos de control avanzados en el futuro.

---

## Referencias

1. [UFactory xArm6 Manual](https://www.ufactory.cc/download/xArm6)  
2. [ros2_control Documentation](https://control.ros.org/)  
3. [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_overview)  
4. [tf2 Tutorials](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Tf2-Main.html)  
5. Denavit, J. and Hartenberg, R.S., ASME J. Appl. Mech., 1955  
6. [RViz - ROS Wiki](http://wiki.ros.org/rviz)
