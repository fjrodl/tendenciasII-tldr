# Optical flow with Simple Opencv

# Teoría

# 📊 Flujo Óptico con OpenCV

## 🎯 ¿Qué es el Flujo Óptico?

- El flujo óptico describe el patrón de movimiento aparente de objetos, superficies y bordes en una escena visual.
- Se estima utilizando **dos fotogramas consecutivos** de un vídeo.
- El objetivo es capturar la **dinámica del movimiento** entre los dos fotogramas.

---

## 🧠 Supuestos del Flujo Óptico

1. **Constancia del brillo**  
   La intensidad de un píxel no cambia entre dos fotogramas.

2. **Movimiento pequeño**  
   El desplazamiento entre fotogramas es pequeño y continuo.

3. **Vecindario consistente**  
   Los píxeles vecinos de un punto tienen movimientos similares.

---

## 🧮 Ecuación del Flujo Óptico

Si \( I(x, y, t) \) es la intensidad de un píxel en el tiempo \( t \), entonces:

\[
I(x, y, t) = I(x + \Delta x, y + \Delta y, t + \Delta t)
\]

Aplicando una expansión de Taylor y asumiendo movimientos pequeños:

\[
\frac{\partial I}{\partial x} \cdot V_x + \frac{\partial I}{\partial y} \cdot V_y + \frac{\partial I}{\partial t} = 0
\]

Donde:

- \( V_x, V_y \): componentes del flujo óptico
- \( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \): gradientes espaciales
- \( \frac{\partial I}{\partial t} \): gradiente temporal

---

## 🔍 Método de Lucas-Kanade (Lucas-Kanade Optical Flow)

El método de **Lucas-Kanade** es una técnica clásica para calcular el flujo óptico entre dos imágenes. Fue propuesto en 1981 y se basa en el supuesto de que el movimiento de los píxeles entre dos fotogramas es **pequeño y coherente dentro de una vecindad local**.

---

### ⚙️ Principios básicos

Lucas-Kanade parte de la ecuación del flujo óptico:

\[
I_x \cdot V_x + I_y \cdot V_y + I_t = 0
\]

Donde:

- \( I_x, I_y \): derivadas espaciales de la imagen (en \(x\) y \(y\))
- \( I_t \): derivada temporal (entre dos imágenes consecutivas)
- \( V_x, V_y \): componentes del flujo óptico (la incógnita)

Esta ecuación no tiene solución única (un punto, dos incógnitas). Lucas-Kanade resuelve este problema utilizando **una ventana de vecindad (por ejemplo, 5x5 píxeles)** y asumiendo que todos los píxeles dentro de esa ventana tienen el mismo movimiento.


### 📐 Interpretación de los términos en la ecuación del flujo óptico

La ecuación diferencial del flujo óptico es:

\[
\frac{\partial I}{\partial x} \cdot V_x + \frac{\partial I}{\partial y} \cdot V_y + \frac{\partial I}{\partial t} = 0
\]

Cada uno de estos términos tiene un significado físico importante:

---

#### 🧭 \( (V_x, V_y) \): Componentes del flujo óptico

- Representan la **velocidad aparente** de movimiento de los píxeles en la imagen.
- \( V_x \): movimiento horizontal (en el eje X) del píxel entre dos fotogramas.
- \( V_y \): movimiento vertical (en el eje Y) del píxel entre dos fotogramas.
- Estas componentes son precisamente lo que queremos estimar con los métodos de flujo óptico.
- El vector \( \vec{V} = (V_x, V_y) \) describe la dirección y magnitud del movimiento.

---

#### 🧮 \( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \): Gradientes espaciales

- Indican cómo cambia la **intensidad de la imagen** en las direcciones X e Y.
- Se calculan derivando la imagen con respecto a sus coordenadas espaciales.
- Un gradiente alto implica un borde o cambio brusco de color o intensidad.
- Estos gradientes nos dicen cómo afecta el desplazamiento espacial al valor de los píxeles.

**Ejemplo:**  
Un borde vertical en la imagen tendrá un gradiente alto en \( \frac{\partial I}{\partial x} \), y bajo en \( \frac{\partial I}{\partial y} \).

---

#### ⏱️ \( \frac{\partial I}{\partial t} \): Gradiente temporal

- Mide el cambio de **intensidad** del mismo píxel entre dos fotogramas consecutivos.
- Esencial para detectar movimiento: si un píxel cambia mucho entre dos imágenes, es probable que haya movimiento.
- Se calcula como la diferencia de intensidad entre el fotograma actual y el anterior:

\[
\frac{\partial I}{\partial t} \approx I(x, y, t+1) - I(x, y, t)
\]

- Si el píxel no ha cambiado de posición, su intensidad debería ser la misma (según el supuesto de **constancia del brillo**).

---

### 🔁 Relación entre todos los términos

- Los gradientes espaciales (\( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \)) indican **cómo varía la imagen localmente**.
- El gradiente temporal (\( \frac{\partial I}{\partial t} \)) indica **cómo ha cambiado con el tiempo**.
- El flujo óptico (\( V_x, V_y \)) es la incógnita que hace que la combinación lineal de estos cambios **sume cero**, bajo el supuesto de que el contenido visual se ha desplazado, pero no modificado.

Esta ecuación se usa como base para estimar el movimiento en muchas técnicas de visión artificial.


---

### 🧮 Solución por mínimos cuadrados

Se forma un sistema sobredeterminado para cada ventana:

\[
A \cdot \vec{v} = \vec{b}
\]

Donde:

- \( A \) contiene las derivadas espaciales (\(I_x, I_y\))
- \( \vec{v} = \begin{bmatrix} V_x \\ V_y \end{bmatrix} \)
- \( \vec{b} = -I_t \)

La solución se obtiene con:

\[
\vec{v} = (A^T A)^{-1} A^T \vec{b}
\]

Este sistema se resuelve para cada punto de interés (por ejemplo, esquinas detectadas con Shi-Tomasi o Harris).

---


## 🔧 Métodos en OpenCV

### 1. `cv2.calcOpticalFlowPyrLK()`

- Método de **Lucas-Kanade**
- Asume movimiento pequeño
- Buen rendimiento en tiempo real
- Ideal para seguimiento de características (features)


## 🧭 Aplicaciones del Flujo Óptico

- Seguimiento de movimiento de objetos

- Estimación de trayectoria

- Compresión de vídeo

- Reconstrucción 3D

- Control de robots y navegación

## 📌 Conclusiones

- El flujo óptico es una herramienta poderosa para comprender el movimiento en escenas visuales.

- OpenCV ofrece herramientas fáciles de usar para aplicar tanto métodos escasos (Lucas-Kanade) como densos (Farneback).

- Su comprensión es clave para aplicaciones de visión artificial en tiempo real.

---

# Test:

   python3 -m venv env
   source env/bin/activate
 
   pip install numpy opencv-python
 

## Demo 1

## Demo 2 

Oficial OpenCV demo.


# References

From https://github.com/npinto/opencv/blob/master/samples/python2/opt_flow.py

From https://github.com/opencv/opencv/tree/4.x/samples/python

From https://viso.ai/deep-learning/optical-flow/

From https://learnopencv.com/optical-flow-in-opencv/





