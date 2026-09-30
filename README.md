# Cubesat
*Develop a dashboard for visualizing telemetry data and snapshot from a CubeSat*

---
## 1. Hardware
The hardware was designed as shown in the bellowing figure:

<div align="center">
  <img src="./Images/hardware.jpg" width="500">
</div>

**Includes**
- 1x Controller esp32-cam
- 1x Sensor MPU6050
- 1x TB6612FNG driver
- 1x motor
- 1x lithium protection battery circuit
- 1x Module DC-DC LX4005
- 2x lithium 18650

This is cubesat that made by myself

<table>
  <tr>
    <td align="center">
      <img src="./Images/cubesat1.jpg" width="100%" />
      <br />
    </td>
    <td align="center">
      <img src="./Images/cubesat2.jpg" width="100%" />
      <br />
    </td>
  </tr>
</table>

## 2. Software
To reduce the computational load on the ESP32, a dashboard application was developed to run on a personal computer instead of directly on the ESP32, as is commonly implemented.

Download this [dashboard](dashboard/main.py) to visual data. 

This video demonstrates how the dashboard data changes when interacting with the CubeSat.

<p align="center">
  <video width="800" controls>
    <source src="./Images/result.mp4" type="video/mp4">
  </video>
</p>