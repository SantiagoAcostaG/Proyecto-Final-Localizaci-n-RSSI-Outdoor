# Proyecto-Final-Localizaci-n-RSSI-Outdoor
# ComDigital-Outdoor

---

# 🛰️ Sistema de Trackeo en Tiempo Real para Carrito de Control Remoto

Este repositorio contiene el código fuente para un sistema distribuido de rastreo de un carrito de control remoto en tiempo real, usando tecnología NRF24L01, sensores inerciales (IMU), pantallas LCD e interconexión mediante módulos Raspberry Pi Pico W y ESP32-CAM.

## 🚗 Descripción del Proyecto

El proyecto permite monitorear en tiempo real la posición y movimiento de un carrito de control remoto utilizando una red de nodos fijos con transceptores NRF24L01 y un nodo móvil montado en el carrito con un sensor inercial (MPU6050). Un concentrador central recopila datos de RSSI (intensidad de señal recibida) desde los nodos y los envía, junto con datos de video y sensores, a una PC central para su procesamiento o visualización.

## 🧩 Componentes del Sistema

* **Nodo móvil**: Raspberry Pi Pico W + MPU6050 + NRF24L01 (envía datos de movimiento y RSSI).
* **Nodos fijos**: 4 Raspberry Pi Pico W + NRF24L01 + LCD para monitoreo local de señales RSSI.
* **Concentrador central**: Raspberry Pi Pico W que recopila datos de RSSI de todos los nodos y los envía a la PC.
* **ESP32-CAM**: Transmite video en tiempo real del carrito vía WiFi.
* **PC Central**: Recibe y visualiza datos del sistema.

## 🧠 Funcionalidades Clave

* Comunicación inalámbrica usando transceptores NRF24L01 vía SPI.
* Lectura de datos de acelerómetro y giroscopio (IMU MPU6050) vía I2C.
* Visualización local en pantallas LCD conectadas a nodos fijos.
* Transmisión de video del carrito mediante ESP32-CAM.
* Envío de todos los datos (video + sensores + RSSI) a la PC central por UART o WiFi.

## 🔧 Requisitos

* Python 3.x (para scripts en PC)
* Thonny / uPyCraft / Arduino IDE (para programar Pico W y ESP32-CAM)
* Bibliotecas: `machine`, `time`, `mpu6050`, `nrf24l01`, `lcd_api`, etc.

## 🚀 Instrucciones de Uso

1. **Flashea los scripts** en cada microcontrolador correspondiente.
2. **Configura la red NRF24L01** asegurando la dirección de cada nodo.
3. **Conecta el ESP32-CAM** a la red WiFi y habilita el stream de video.
4. **Ejecuta el script en la PC central** para visualizar y procesar los datos.

## 📸 Diagrama del Sistema

![Esquemático del sistema](./esquematico_proyecto.jpeg)

## 📌 Estado del Proyecto

✅ Diseño funcional completo
🔄 Mejoras en optimización de código y visualización en curso

## 🤝 Contribuciones

¡Bienvenid@s! Puedes enviar pull requests o abrir issues para sugerencias o mejoras.

---
