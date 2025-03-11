# 📌 Análisis del fichero de configuración del EKF en `robot_localization`

Este fichero de configuración pertenece a **`robot_localization`**, específicamente a un **filtro EKF (Extended Kalman Filter)**. Su función es fusionar datos de diferentes sensores (odometría, IMU, GPS, etc.) para proporcionar una estimación más precisa de la posición y orientación del robot en **ROS 2**.

[Fichero de configuración del rover](https://github.com/mgonzs13/ros2_rover/tree/humble/rover_localization/config)
---

## **📌 1. Publicación de TFs**
```yaml
publish_tf: true
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
```
🔹 **`publish_tf: true`** → Publica la transformación **`map → odom → base_link`**.  
🔹 **Frames principales:**  
- **`map_frame: map`** → Es el frame de referencia global (corregido con SLAM o AMCL).  
- **`odom_frame: odom`** → Es el frame de referencia local (proporcionado por odometría).  
- **`base_link_frame: base_link`** → Es el frame del robot.  
- **`world_frame: odom`** → Define el marco de referencia mundial (normalmente `odom` o `map`).  

✅ **Propósito:**  
- Mantiene una estructura consistente de transformaciones **TF2**.  
- `map → odom` permite corregir la deriva de la odometría con localización global.  
- `odom → base_link` proporciona una odometría continua y suave.  

---

## **📌 2. Configuración de Sensores (Odometría e IMU)**
### **🔹 Odometría (`odom0`)**
```yaml
odom0: odom_rgbd
odom0_config: [
    true, true, true,   # Posición (X, Y, Z)
    false, false, true, # Orientación (roll, pitch, yaw)
    true, true, true,   # Velocidad lineal (X, Y, Z)
    false, false, false, # Velocidad angular (roll, pitch, yaw)
    false, false, false # Aceleración (X, Y, Z)
  ]
odom0_differential: true
odom0_relative: false
```
✅ **Significado:**  
- Se usa un **sensor de odometría (`odom_rgbd`)**, posiblemente basado en visión (como un sensor RGB-D).  
- Solo se considera la **posición en X, Y, Z** y el **yaw (orientación en el plano 2D)**.  
- Se usa la **velocidad lineal en X, Y, Z**, pero no la velocidad angular.  
- **`odom0_differential: true`** → La odometría se interpreta de forma diferencial (basada en cambios en la posición).  

---

### **🔹 IMU (`imu0`)**
```yaml
imu0: imu
imu0_config: [
    false, false, false, # Posición
    true, true, false,   # Orientación (roll, pitch, yaw)
    true, true, true,    # Velocidad lineal (X, Y, Z)
    true, true, true,    # Velocidad angular (roll, pitch, yaw)
    false, false, false  # Aceleración lineal
  ]
imu0_differential: false
imu0_relative: true
```
✅ **Significado:**  
- Se usa un **sensor IMU** (`imu`).  
- Solo se toma en cuenta la **orientación (roll, pitch)**, pero no el `yaw`.  
- Se usa la **velocidad lineal y angular** para mejorar la estimación de movimiento.  
- **`imu0_relative: true`** → Los datos de la IMU se interpretan de manera relativa (respecto al frame `base_link`).  

---

## **📌 3. Calidad de Servicio (QoS)**
```yaml
qos_overrides./odom.publisher.reliability: reliable
qos_overrides./accel.publisher.reliability: reliable
```
✅ **Propósito:**  
- Se establece una configuración **reliable** en los tópicos de odometría y aceleración.  
- Esto garantiza que los datos lleguen completos y sin pérdidas en redes con comunicación inestable.  

---

## **📌 4. Tiempo y Diagnósticos**
```yaml
use_sim_time: false
print_diagnostics: true
debug: false
```
✅ **Propósito:**  
- **`use_sim_time: false`** → Usa el tiempo real del sistema en lugar del tiempo de simulación.  
- **`print_diagnostics: true`** → Publica información de diagnóstico útil para debugging.  
- **`debug: false`** → No imprime información extra en la consola.  

---

## **📌 5. Covarianza de Ruido del Proceso**
```yaml
process_noise_covariance: [ ... ]
```
✅ **Propósito:**  
- Define la incertidumbre en la estimación del estado del robot.  
- Valores más altos indican mayor incertidumbre en la estimación del filtro EKF.  

---

## **📌 6. Covarianza de Estimación Inicial**
```yaml
initial_estimate_covariance: [ ... ]
```
✅ **Propósito:**  
- Define cuánta confianza tiene el sistema en la estimación inicial del estado del robot.  
- Valores más bajos indican una estimación inicial más precisa.  

---

## **📌 🔥 Resumen General**
| **Parámetro** | **Función** |
|--------------|------------|
| **`publish_tf: true`** | Publica `map → odom → base_link` en **TF2**. |
| **`odom0: odom_rgbd`** | Usa odometría basada en cámara RGB-D. |
| **`imu0: imu`** | Usa datos de la IMU para mejorar la estimación. |
| **`odom0_differential: true`** | Usa odometría diferencial en la estimación. |
| **`imu0_relative: true`** | Usa datos relativos de la IMU. |
| **`qos_overrides`** | Configura la fiabilidad de los mensajes de odometría/aceleración. |
| **`process_noise_covariance`** | Define la incertidumbre en la estimación del EKF. |
| **`initial_estimate_covariance`** | Define la confianza en el estado inicial del robot. |

---

## 🎯 **Conclusión**
Este archivo configura un **EKF** para fusionar **odometría RGB-D y datos de IMU** en un **sistema de navegación en ROS 2**, asegurando que el robot tenga una **estimación precisa y robusta de su estado en TF2**. 🚀
