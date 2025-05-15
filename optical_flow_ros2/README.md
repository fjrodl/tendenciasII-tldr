# 🧠 Práctica ROS 2: Odometría Visual con Optical Flow

## 🎯 Objetivo

Desarrollar un sistema de odometría visual básico utilizando flujo óptico en ROS 2. El sistema debe estimar la posición y orientación del robot en el plano (2D) a partir del análisis del movimiento aparente en una secuencia de imágenes capturadas por una cámara.

---

## 📚 Motivación

En escenarios donde no se dispone de sensores de posicionamiento precisos (como GPS, IMU o encoders), la visión por computador ofrece una alternativa de bajo costo para estimar el movimiento de un robot.

El Optical Flow permite detectar y cuantificar el desplazamiento aparente de píxeles entre imágenes sucesivas. Aunque no proporciona una escala métrica por sí mismo, puede utilizarse como base para una estimación incremental de la trayectoria del robot.

Esta práctica tiene como objetivo:
- Comprender los principios básicos del flujo óptico.
- Aplicar técnicas de seguimiento de puntos (Lucas-Kanade).
- Estimar una transformación afín entre frames para calcular traslación y rotación.
- Publicar esta información como odometría en ROS 2.
- Visualizar resultados en RViz y herramientas de ROS.

---

## 🔧 Pasos de desarrollo

### 1. Crear el paquete ROS 2
Se genera un paquete Python con `ros2 pkg create`, siguiendo el estilo estándar de ROS 2. Se añaden los archivos necesarios: nodo principal, launch files y configuración.

### 2. Implementar el flujo óptico
Se utiliza OpenCV para:
- Detectar características (puntos de interés).
- Hacer seguimiento de esos puntos entre imágenes sucesivas con Lucas-Kanade.
- Calcular la transformación afín entre los conjuntos de puntos.

### 3. Estimar odometría
La transformación afín (traslación + rotación) permite estimar la posición y orientación relativa del robot en el plano 2D. Esta información se acumula y se publica como un mensaje `nav_msgs/Odometry`.

### 4. Publicar transformaciones TF
Se utiliza `tf2_ros` para emitir un `TransformStamped` que relaciona el marco `odom` con el marco `base_link`. Esto permite a RViz representar la trayectoria del robot correctamente en el espacio.

### 5. Publicar la imagen visual
La imagen con vectores de flujo se convierte en un `sensor_msgs/Image` y se publica en un tópico para poder visualizarla con `rqt_image_view` o `image_view`.

### 6. Filtrar el ruido de flujo óptico
Para evitar que pequeños cambios visuales se interpreten como desplazamientos reales, se descartan los vectores de flujo cuya magnitud es inferior a un umbral configurable.

### 7. Visualizar en RViz
Se crea un launch file que lanza tanto el nodo como RViz, configurado con vistas para:
- La imagen del flujo óptico.
- El frame `base_link`.
- La trayectoria sobre `/odom`.

---

## 📈 Limitaciones conocidas

- No se corrige la **deriva acumulativa**.
- El sistema no tiene información de escala real.
- Cambios de iluminación o textura afectan el flujo.
- No se estima profundidad ni se maneja el movimiento fuera del plano.

---

## 🧩 Extensiones posibles

- Fusión con IMU mediante `robot_localization`.
- Estimación de la escala usando una cámara estéreo o referencias visuales.
- Uso de algoritmos SLAM como `ORB-SLAM2` o `RTAB-Map`.
- Mejora de la robustez con detectores como ORB o SIFT.
- Aplicación en drones o robots móviles reales.

---

# 🔗 Del Flujo Óptico a la Odometría Visual y SLAM

## 1️⃣ Flujo Óptico: el punto de partida

El **flujo óptico** estima cómo se mueven los píxeles entre dos imágenes consecutivas.  
Es útil para:

- Detección de movimiento
- Seguimiento de puntos
- Estimación de velocidad aparente

Sin embargo:

- Solo proporciona **movimiento relativo local**
- No conoce la **geometría 3D** ni la posición absoluta
- No mantiene una **trayectoria acumulativa estable**

---

## 2️⃣ Odometría Visual: estimar el movimiento de la cámara

La **odometría visual (Visual Odometry, VO)** es un paso más allá del flujo óptico.

Su objetivo es:

> Estimar la **pose (posición y orientación)** de la cámara a lo largo del tiempo, **integrando información visual entre múltiples fotogramas**.

### 📷 ¿Cómo lo hace?

- Detecta puntos clave (features) en imágenes sucesivas
- Establece correspondencias (matches) entre esos puntos
- Usa triangulación o PnP para estimar el movimiento entre imágenes
- Integra estos desplazamientos en una trayectoria de la cámara

📌 Algunos métodos de VO usan directamente **optical flow** (por ejemplo, KLT tracker) para seguir puntos clave entre fotogramas.

---

### 🔍 Relación entre Optical Flow y Visual Odometry

| Concepto         | Optical Flow                      | Visual Odometry                       |
|------------------|-----------------------------------|----------------------------------------|
| Entrada          | Dos imágenes consecutivas         | Secuencia de imágenes                 |
| Resultado        | Movimiento de píxeles             | Pose de la cámara                     |
| Tipo de salida   | Campo de vectores 2D              | Transformación 3D (R, t)              |
| Dependencias     | Gradientes locales                | Geometría de la cámara                |
| Uso de features  | Opcional (denso o escaso)         | Generalmente usa puntos clave         |
| Precisión        | Local                             | Global (a corto plazo)                |

---

## 3️⃣ ¿Dónde entra SLAM? (ORB-SLAM, RTAB-Map)

El problema de la odometría visual es que **acumula error** con el tiempo (deriva).  
Ahí es donde entra **SLAM (Simultaneous Localization and Mapping)**:

> SLAM no solo estima la trayectoria de la cámara (como VO), sino que también **construye un mapa** del entorno y corrige errores mediante **cerrado de bucles** y **relocalización**.

### 🧱 ¿Qué añade SLAM sobre VO?

- 🗺️ **Mapa del entorno**
- 🔁 **Cierre de bucles** para reducir error acumulado
- 📍 **Re-localización** en zonas ya visitadas
- 🔧 **Optimización global** de la trayectoria

---

## 🧠 De flujo óptico a mapeo robusto: la progresión lógica

```text
Flujo Óptico
   ↓
Odometría Visual
   ↓
SLAM (ej. ORB-SLAM, RTAB-Map)
