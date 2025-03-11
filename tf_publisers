# 📌 Componentes que publican las TFs necesarias para la navegación en ROS 2

En un sistema de navegación en **ROS 2**, varios componentes publican las transformaciones (**TFs**) necesarias para que el robot pueda moverse correctamente en su entorno. A continuación, se enumeran los principales componentes que suelen publicar **TFs** y su función en el sistema.

---

## **1️⃣ `robot_state_publisher`**
🔹 **Publica:**  
- Las transformaciones de los enlaces (`links`) del robot basadas en su modelo **URDF/Xacro**.  
- Se basa en los datos de las articulaciones (`/joint_states`).

🔹 **Ejemplo de TFs publicadas:**  
```
base_link → arm_link  
base_link → camera_link  
base_link → wheel_left  
```

🔹 **¿Por qué es importante?**  
- Permite visualizar la cinemática del robot en **Rviz**.  
- Publica las relaciones entre las partes del robot de forma continua.  

---

## **2️⃣ `joint_state_publisher`**
🔹 **Publica:**  
- El estado de las articulaciones del robot (`/joint_states`).  
- Se usa junto con **`robot_state_publisher`** para actualizar la cinemática del robot.

🔹 **Ejemplo de TFs publicadas:**  
```
base_link → arm_joint  
```

🔹 **¿Por qué es importante?**  
- Es clave para robots con **brazos robóticos** o mecanismos articulados.  
- Ayuda a visualizar y simular la posición de las articulaciones.

---

## **3️⃣ `odom` (odometría del robot)**
🔹 **Publica:**  
- La transformación entre `odom` y `base_link` (`odom → base_link`).
- Basada en sensores como **encoders, IMU o un filtro de Kalman**.

🔹 **Ejemplo de TFs publicadas:**  
```
odom → base_link
```

🔹 **¿Por qué es importante?**  
- Proporciona una referencia **suave y continua** para la navegación local.  
- Permite planificar trayectorias a corto plazo sin cambios bruscos.  

---

## **4️⃣ `map` (localización y SLAM)**
🔹 **Publica:**  
- La transformación entre `map` y `odom` (`map → odom`).  
- Generalmente publicada por **SLAM** (`slam_toolbox`, `cartographer`) o por **AMCL**.

🔹 **Ejemplo de TFs publicadas:**  
```
map → odom
```

🔹 **¿Por qué es importante?**  
- Proporciona una referencia **global y corregida** para la navegación.  
- Corrige el error de la odometría acumulada a largo plazo.

---

## **5️⃣ `laser_scan_matcher` o `scan_to_map`**
🔹 **Publica:**  
- La transformación `odom → base_link` basada en **LIDAR**.  
- Se usa como alternativa a la odometría tradicional.

🔹 **Ejemplo de TFs publicadas:**  
```
odom → base_link
```

---

## **6️⃣ `tf_static` (transformaciones fijas)**
🔹 **Publica:**  
- Transformaciones **estáticas** entre elementos fijos del robot (ejemplo: `camera_link → base_link`).

🔹 **Ejemplo de TFs publicadas:**  
```
base_link → camera_link  
base_link → imu_link  
```

---

## **7️⃣ `robot_localization` (fusión de sensores)**
🔹 **Publica:**  
- La transformación entre `odom` y `base_link`, basada en la fusión de **IMU, GPS y encoders**.  
- También puede actualizar `map → odom` si recibe datos de GPS.

🔹 **Ejemplo de TFs publicadas:**  
```
map → odom  
odom → base_link
```

---

## **8️⃣ `move_base` / `nav2` (planificación de movimiento)**
🔹 **Publica:**  
- Transformaciones necesarias para la navegación.  
- En algunos casos, puede corregir la posición del `base_link`.

🔹 **Ejemplo de TFs publicadas:**  
```
base_link → footprint  
base_link → goal_pose  
```

---

## **9️⃣ `cartographer` o `slam_toolbox` (mapeo y localización simultáneos)**
🔹 **Publica:**  
- `map → odom`, basado en datos de SLAM.

🔹 **Ejemplo de TFs publicadas:**  
```
map → odom
```

---

## **🔟 `AMCL` (localización probabilística)**
🔹 **Publica:**  
- `map → odom`, basado en la localización dentro de un mapa conocido.

🔹 **Ejemplo de TFs publicadas:**  
```
map → odom
```

---

## **📌 Resumen de TFs en un sistema de navegación**
| **Componente**          | **TFs publicadas**         | **Función** |
|------------------------|--------------------------|-------------|
| `robot_state_publisher` | `base_link → links` | Define la cinemática del robot desde URDF. |
| `joint_state_publisher` | `base_link → joint` | Publica el estado de las articulaciones. |
| `odom` | `odom → base_link` | Publica la odometría del robot. |
| `map` (AMCL/SLAM) | `map → odom` | Corrige la odometría usando un mapa. |
| `laser_scan_matcher` | `odom → base_link` | Usa LIDAR para mejorar la odometría. |
| `tf_static` | `base_link → sensores` | Publica transformaciones estáticas. |
| `robot_localization` | `map → odom`, `odom → base_link` | Fusiona sensores (IMU, GPS, encoders). |
| `move_base` / `nav2` | `base_link → goal_pose` | Publica información de navegación. |
| `cartographer / slam_toolbox` | `map → odom` | Publica localización basada en SLAM. |
| `AMCL` | `map → odom` | Publica localización probabilística. |

---

## 🎯 **Conclusión**
- **Los nodos de navegación (`move_base`, `nav2`) dependen de `TF2` para planificar rutas.**
- **El `robot_state_publisher` y `tf_static` aseguran que la estructura del robot sea correcta.**
- **`map → odom` se usa para corregir la odometría en función de la localización global.**
- **`odom → base_link` se usa para la navegación local y debe ser una transformación suave.**

Así es como los diferentes componentes publican las **TFs** necesarias para que un robot navegue de manera eficiente en su entorno en **ROS 2**. 🚀
