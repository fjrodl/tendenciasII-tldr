# ¿Por qué el REP 105 considera el `map` y el `odom` estáticos?

El **REP 105** ("Coordinate Frames for Mobile Platforms") define la convención de transformaciones de referencia en robots móviles dentro de ROS y especifica que los frames `map` y `odom` se consideran **estáticos** en sus propios contextos, aunque en la práctica la relación entre ellos cambia con el tiempo. Veamos por qué:

---

## 1. `map` es estático en relación con el mundo
- `map` representa una referencia **global y fija** en la que se sitúa el entorno del robot.  
- Suponemos que el mundo no cambia constantemente en la localización del robot, por lo que `map` se mantiene fijo.
- Ejemplos de fuentes para este frame:
  - **SLAM** (mapeo simultáneo y localización).
  - **Mapas predefinidos** con AMCL (localización probabilística).
  - **GPS** en entornos exteriores.

### 🚨 En la práctica:
Aunque `map` es fijo, **su relación con `odom` cambia**, ya que la localización del robot puede corregir el error acumulado de la odometría.

---

## 2. `odom` es estático en relación con la odometría
- `odom` es una referencia **local y suave**, derivada de los sensores de odometría (encoders, IMU, filtros de Kalman).
- **Se considera fijo**, porque en cortos periodos de tiempo la odometría es precisa y no hay cambios bruscos.
- **No sufre discontinuidades**, lo que evita saltos en la navegación local del robot.

### 🚨 En la práctica:
- Aunque `odom` no cambia en sí mismo, su transformación con `map` **sí puede cambiar** para corregir errores de odometría cuando se usa un sistema de localización global.
- Por ejemplo, `amcl` o `robot_localization` pueden reajustar la relación `map → odom` a medida que se detectan correcciones en la posición.

---

## ¿Por qué REP 105 los considera "estáticos"?
1. **Para evitar movimientos abruptos en el robot.**  
   - Si `map` o `odom` fueran dinámicos en su propio contexto, un cambio repentino en `odom` afectaría la planificación local y causaría movimientos erráticos.
  
2. **Para garantizar estabilidad en la navegación.**  
   - `odom` permite que la navegación local sea predecible.  
   - `map` proporciona una referencia global sin cambios inesperados.

3. **Porque la relación `map → odom` puede cambiar, pero no dentro de su propio sistema de referencia.**  
   - `map` es una referencia fija para localización.  
   - `odom` es una referencia fija para odometría.  
   - **La transformación entre ambos es la que cambia dinámicamente.**

---

## Conclusión
✅ `map` y `odom` son **fijos en su propio contexto** para garantizar estabilidad en la navegación.  
✅ **La relación entre ellos puede cambiar dinámicamente**, ajustando la posición del robot según la localización global.  
✅ Esto permite que el robot tenga una referencia local suave (`odom`) y una referencia global corregida (`map`), evitando saltos bruscos en la navegación.

---
---
# ¿Por qué hace falta `odom` entre `world/map` y `base_footprint` en las TFs2 de ROS?

En **ROS 2**, la transformación entre `world/map` y `base_footprint` requiere un **frame intermedio**, generalmente llamado `odom`, debido a la separación de referencias globales y locales en la navegación de robots. Aquí están las razones clave:

---

## 1. Separación de Referencias Globales y Locales
- **`map` (o `world`)**: Es un **frame global estático** usado para la localización absoluta del robot dentro del entorno. Se obtiene a partir de sensores como **LIDAR**, **SLAM**, o **GPS**.
- **`odom`**: Es un **frame local** que proporciona una referencia de movimiento **suave y continua**, derivada de la odometría del robot (ruedas, IMU, etc.).
- **`base_footprint`**: Representa la posición real del robot, sin considerar la altura, con respecto a la odometría.

---

## 2. Reducción del Ruido y Discontinuidades
- La odometría es muy precisa a corto plazo, pero acumula error con el tiempo (**drift**).
- El sistema de localización (SLAM, AMCL, GPS, etc.) puede corregir este error, pero sus actualizaciones pueden ser **irregulares** y **discontinuas**.
- Al introducir `odom` entre `map` y `base_footprint`, evitamos que el robot tenga movimientos bruscos o teletransportaciones inesperadas cuando la localización global se actualiza.

---

## 3. Combinación de Datos de Sensores
- `odom` se basa en datos de **encoders, IMU y filtros de Kalman**.
- `map` se corrige usando **SLAM o localización global**.
- La transición entre estos dos frames se maneja mediante **un estimador de estado**, como `robot_localization`, que combina ambos tipos de información.

---

## 4. Facilita la Navegación y el Control
- La planificación **local** (evitar obstáculos) usa `odom`.
- La planificación **global** (ruta completa) usa `map`.
- La combinación de ambos garantiza una navegación fluida.

---

## Conclusión
✅ La transformación `map → odom → base_footprint` es esencial porque permite:  
- **Movimientos suaves sin saltos bruscos.**  
- **Corrección del error acumulado en la odometría.**  
- **Separación entre referencia global (`map`) y local (`odom`).**  
- **Mejora de la estabilidad en la navegación.**  

En sistemas donde no se usa localización global, el frame `map` puede no ser necesario, pero `odom` sigue siendo clave para garantizar una odometría continua y útil.

