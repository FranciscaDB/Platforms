# 3D Printing Instructions

This repository contains the recommended slicing configuration for 3D printing components used in the NVIDIA JetBot project.
---

## ✅ Recommended Cura Slicer Settings

To achieve optimal quality when printing the parts, please use the following slicing configuration in Cura:

### Quality
- **Layer Height**: `0.15 mm`

### Shell
- **Wall Thickness**: `0.7 mm`
- **Top/Bottom Thickness**: `0.75 mm`

### Infill
- **Infill Density**: `18%`

### Speed
- **Print Speed**: `60 mm/s`
- **Wall Speed**: `60 mm/s`
- **Top/Bottom Speed**: `30 mm/s`
- **Travel Speed**: `150 mm/s`
- **Initial Layer Speed**: *Auto (Calculated by Cura)*

### Cooling
- **Minimum Layer Time**: `5 s`
- **Minimum Speed**: `10 mm/s`

### Support
- **Support Overhang Angle**: `55°`
- **Support Horizontal Expansion**: `0.0 mm`

### Build Plate Adhesion
- **Brim Width**: `5 mm`

---

## 🔧 Additional Printing Tips

- Ensure your build plate is clean and properly leveled before starting the print.
- Enable part cooling (fan) for best results with overhangs and small features.
- Use a **brim** to improve bed adhesion and reduce warping.
- Recommended material: **PLA**  
  Typical PLA settings:  
  - Nozzle Temperature: `200–210°C`  
  - Bed Temperature: `60°C`

---

## 📎 Source and Attribution

The caster ball base and shroud models are part of the official JetBot hardware design by **NVIDIA**.

You can find more information and download the original files at:  
👉 [NVIDIA JetBot – 3D Printed Parts](https://jetbot.org/master/3d_printing.html)
