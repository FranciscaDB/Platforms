<p align="center">
  <img src="jetros.png" alt="image" width="400"/>
</p>

### Bill of Materials
| Component                                  | Link               |
|--------------------------------------------|--------------------|
| NVIDIA Jetson Orin Nano Dev Kit            | https://developer.nvidia.com/buy-jetson?product=all&location=CA                    |
| Waveshare UPS Module for Jetson Orin       | https://www.waveshare.com/ups-power-module-c.htm                   |
| 21700 Li battery x3                        |                    |
| Slamtec RPLIDAR C1                         | https://www.amazon.ca/ZICZNT-scanning-Positioning-Navigation-Avoidance/dp/B0DDKZV8HZ                   |
| Akyta AKT-N5-1 HD Webcam                   | https://www.amazon.ca/Desktop-with110-Degree-Digital-Microphone-Recording/dp/B07Z2GZTYY/ref=sr_1_2_sspa?dib=eyJ2IjoiMSJ9.pXeG26BTC5hvFJ40Ct2PhaFX4HutAoiicXlK811VYzKPDkw9C7hKWKjAtYA9qXG6E2Y-nb9XZDwy3LrC5s7bS0bC-1IXs6iu9vnR4Ysq6F9xQKCA3Hl1F1GdtEvFxKsjH0swUyCYHS51I8Z6yXrJiqjCCSG2PTz1ZDnCY69iqUKmfvBzJzODeViOflWfKkzodqKcx1N0-cmGndViQBaTzQADP-gzqSFkNssngn3knH5gSWfPr7-UrPR2ySG8ddfh344lwGLNXQuElL6RRHr0kz47Xkw-KCitanbHAPlFJeQ.k5V0GyATvKPSM1bxG-0Fjj7GKxNpIYP_dT9ij5JJ1wU&dib_tag=se&keywords=Akyta+AKT-N5-1+HD+Webcam&qid=1750630632&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1                   |
| GA12-N20 DC Encoder Motor x2               | https://www.amazon.ca/Reduction-Rotating-GA12-N20-Encoder-Permanent/dp/B0CC55GCMM?th=1                   |
| Adafruit FeatherWing Motor Driver          | https://www.adafruit.com/product/2927?srsltid=AfmBOooZTwYZpVQFFOb1u36j2i_Kcq3z-mYKjlnS9TZ4RGsDC-pft_L8                   |
| AsperX Power Bank 10000mAh                 |                    |


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
|                  | M1           | M1-M2     |           |
|                  | M2           |           | M1-M2     |

<p align="center">
<img src="pins_orin.png" width="300">
<img src="encoder.jpg" width="300">
<img src="driver.png" width="300">
</p>
