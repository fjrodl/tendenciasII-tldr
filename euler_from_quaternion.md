# Explicación de `euler_from_quaternion`

La función `euler_from_quaternion` convierte una orientación representada en **cuaterniones** a **ángulos de Euler** (roll, pitch, yaw). Esta conversión es necesaria en ROS 2 porque los datos de orientación en `Odometry` y `Transform` se representan en cuaterniones, pero en muchas aplicaciones es más útil trabajar con ángulos de Euler, especialmente el **yaw** para robots móviles.

## ¿Por qué usar cuaterniones en ROS 2?

Los cuaterniones son una forma matemática de representar rotaciones en 3D sin sufrir los problemas de los ángulos de Euler, como las **singularidades (gimbal lock)**. Sin embargo, para robots móviles, generalmente solo nos interesa el **ángulo de giro en el plano XY (yaw)**.

## Desglose de la función

```python
def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion

    # Calcula yaw (rotación en el plano XY)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return 0, 0, yaw
```

## Explicación paso a paso

1. **Extraer componentes del cuaternión**:

    ```python
    x, y, z, w = quaternion
    ```
    - `x, y, z` representan la parte vectorial del cuaternión.
    - `w` es la parte escalar.

2. **Calcular el yaw** (rotación alrededor del eje Z):

    ```python
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    ```
    - `siny_cosp` y `cosy_cosp` son expresiones trigonométricas extraídas de la conversión de cuaterniones a ángulos de Euler.
    - `np.arctan2(siny_cosp, cosy_cosp)` calcula directamente el ángulo **yaw** en radianes.

3. **Retorno de la función**:

    ```python
    return 0, 0, yaw
    ```
    - Devuelve `roll = 0`, `pitch = 0` (no los calculamos porque no son necesarios en robots móviles).
    - Devuelve `yaw`, que es lo que realmente necesitamos.


## ¿Cómo se usa en ROS 2?

Cuando recibimos un mensaje de `Odometry` o `Transform`, podemos extraer el cuaternión y obtener el yaw de la siguiente manera:

```python
def odom_callback(self, msg):
    quaternion = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    self.yaw = self.euler_from_quaternion(quaternion)[2]
```
Aquí, self.yaw almacena la orientación del robot en radianes, lista para ser utilizada en cálculos de control o navegación.

## Ejemplo práctico

Supongamos que un robot móvil necesita conocer su orientación en un entorno 2D. Dado que en ROS 2 la orientación se representa mediante cuaterniones, es necesario convertirlos a ángulos de Euler para obtener el **yaw**, que indica la dirección del robot en el plano XY.

A continuación, se presentan algunos ejemplos prácticos de conversión:

1. **El robot apunta hacia el este (0° o 0 radianes)**:
    - Cuaternión: `(x=0, y=0, z=0, w=1)`
    - Resultado esperado: `yaw = 0.0 rad`

2. **El robot apunta hacia el norte (90° o π/2 radianes)**:
    - Cuaternión: `(x=0, y=0, z=0.707, w=0.707)`
    - Resultado esperado: `yaw ≈ 1.57 rad`

3. **El robot apunta hacia el oeste (180° o π radianes)**:
    - Cuaternión: `(x=0, y=0, z=1, w=0)`
    - Resultado esperado: `yaw ≈ 3.14 rad`

Estos valores de `yaw` pueden utilizarse en algoritmos de navegación y control para ajustar la dirección del robot en su desplazamiento. 

