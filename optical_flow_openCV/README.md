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





