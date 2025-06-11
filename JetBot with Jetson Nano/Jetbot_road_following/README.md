# Usage Explanation

It is strongly recommended to first play with the various examples that come by default with the JetBot image.

## Road Following 

In short, the idea is to execute a flow of code:

1. 1_data_collection.ipynb

2. 2_train_model.ipynb

3. Live demo
   - Option 1: 3a_live_demo.ipynb -> To control just one JetBot
   - Option 2: 3b_1_live_demo_build_trt.ipynb -> Optimize the model using TensorRT
   
   If you choose **Option 2**:
   - Option 2.1: 3b_2_live_demo_trt.ipynb -> To control just one JetBot with a optimized model
   - Option 2.2: Platooning_Leader.ipynb or Platooning _NX.ipynb -> These codes use MQTT to enable communication 

Platooning_Leader.ipynb runs on the jetbot that will be controlled through the GUI, while Platooning_NX.ipynb, with X = jetbot number, will be executed on the jetbot that will receive inputs via MQTT.

## How to configure the parameters in Platooning_*.ipynb

Both codes have a section called "MQTT configuration." There, the **MQTT Broker** and the **MQTT Topic** should be added and be the same. It is necessary to install the paho-mqtt package beforehand to use this functionality. Additionally, it is necessary to have a device running the Mosquitto server. This can be a JetBot or a computer with Ubuntu (connected to the same local network), and the IP of this will be use in **MQTT Broker**.

To install Mosquitto service:
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```

Start the Mosquitto service manually (before run any code Platooning_*.ipynb):
```bash
sudo systemctl start mosquitto
```

Optionally, enable the automatic start of the service at boot.
```bash
sudo systemctl enable mosquitto
```

Verify that the service is running:
```bash
sudo systemctl status mosquitto
```

This should be executed in the terminal. If it's a JetBot, it should be run via Serial (e.g., PuTTY) or by SSH using SSH user@IP.
