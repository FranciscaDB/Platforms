The next sections must be done in order.

## 1. Install JetPack 5.1.4

It is important to use JetPack 5.1.4, as newer versions make it more difficult to use the GPIOs.

To install this specific JetPack version, you need to use the JetPack SDK Manager on a host computer running Ubuntu 20.04. Other versions of Ubuntu may not provide access to JetPack 5.1.4 in the SDK Manager.

Some tips for this step:

- During this process, it is ideal to have the Jetson Orin Nano connected to a monitor, keyboard, and mouse.  
- Perform the JetPack installation in a place with good internet access and simple connection settings (for example, a home router with a basic username and password, not a university network).  
- The installation may take several hours, and the computer must remain connected to the Jetson Orin Nano during the entire process.  
- When the option appears, select "Install OS runtime" and then provide the IP address and username to install the tools via SSH.



## 2. Install ROS 2 foxy

Follow all the steps from the Installation (Ubuntu - Debian) section at:

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

After completing the installation, continue with the Tutorials section. Once you are able to run the turtlesim examples, your ROS 2 environment will be mostly ready.

## 3. Install PyTorch 2.1 and TorchVision 0.16.1

https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

PyTorch mus be downloaded from https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl and this file must be installed on Jetson Orin nano as:

```bash
pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
```

Then, install it on the Jetson Orin Nano with:

```bash
pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl)
```

TorchVision can be install following: 

```bash
$ sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
$ git clone --branch <version> https://github.com/pytorch/vision torchvision   # see below for version of torchvision to download
$ cd torchvision
$ export BUILD_VERSION=0.x.0  # where 0.x.0 is the torchvision version  
$ python3 setup.py install --user
$ cd ../  # attempting to load torchvision from build dir will result in import error
$ pip install 'pillow<7' # always needed for Python 2.7, not needed torchvision v0.5.0+ with Python 3.6
```

## 4. Set up Jupyter Lab 4 and jupyter_clickable_image_widget (neccesary for collect data script)
```bash
pip install jupyterlab==4.0.10 ipywidgets
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g yarn
git clone https://github.com/jaybdub/jupyter_clickable_image_widget
cd jupyter_clickable_image_widget
cd js
yarn install
yarn add @jupyter-widgets/controls
yarn run build:prod
cd ..
pip install .
jupyter lab build
jupyter labextension install js
```

## 5. Install torch2trt
```bash
git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
python3 setup.py install --user
```
