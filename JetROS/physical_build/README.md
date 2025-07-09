
### Connections:
| Jetson Orin Nano | Motor Driver | Encoder L | Encoder R |
|------------------|--------------|-----------|-----------|
| 1                | 3V3          |           |           |
| 2                |              | VCC       |           |
| 3                | SDA          |           |           |
| 4                |              |           | VCC       |
| 5                | SCL          |           |           |
| 6                | GND          |           |           |
| 9                |              |           | GND       |
| 11               |              |           | C1        |
| 12               |              |           | C2        |
| 14               |              | GND       |           |
| 15               |              | C1        |           |
| 16               |              | C2        |           |
| N/A              | M1           | M1-M2     |           |
| N/A              | M2           |           | M1-M2     |

<p align="center">
<img src="pins_orin.png" width="300">
<img src="encoder.jpg" width="300">
<img src="driver.png" width="300">
</p>
