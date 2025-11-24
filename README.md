# RoboDriver

Docs: https://baai-ei-data.github.io/RoboDriver-Doc/

## Support Robot

P1:
```
aloha_v1, operating_platform <= 0.1.5
SO101_v1, operating_platform <= 0.1.5
galaxea_v1, operating_platform <= 0.1.5
realman_v1, operating_platform <= 0.1.5
dexterous_hand_v1, operating_platform <= 0.1.5
```

P2:
```
pika_v1, operating_platform <= 0.1.3
galbot_g1, operating_platform <= 0.1.3
leju_kuavo4p, operating_platform <= 0.1.3
agibot_a2d, operating_platform <= 0.1.3
```

P3:
```
adora_v1, Not Support
ruantong_v1, Not Support
```

## Start
creat conda env

```sh
conda create --name robo_driver python==3.10
```

activate conda env

```sh
conda activate robo_driver
```

install this project

```sh
pip install -e .
```

**install pytorch, according to your platform**

```sh
# ROCM 6.1 (Linux only)
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/rocm6.1
# ROCM 6.2.4 (Linux only)
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/rocm6.2.4
# CUDA 11.8
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu118
# CUDA 12.4
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu124
# CUDA 12.6
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu126
# CPU only
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cpu
```

## TODO

- Validate SO101 in new Code
- Function: Compare server code version
- Save device info

```
    "flask-cors>=5.0.0,<7.0.0",

    "pynput>=1.8.1,<2.0.0",
    "python-socketio>=5.13.0,<6.0.0",
    "websocket-client>=1.8.0,<2.0.0",
    "schedule>=1.2.2,<2.0.0",
    "pillow>=10.4.0,<12.0.0",

    "jsonlines>=4.0.0,<5.0.0",

    "gymnasium>=1.1.1,<2.0.0",
    "einops>=0.8.1,<0.9.0",

    "diffusers>=0.33.1,<0.34.0",
    "pymunk>=6.11.1,<8.0.0",

    "deepdiff>=8.4.2,<9.0.0",
    "zarr>=2.16.1,<4.0.0",
    "numba>=0.58.1,<0.62.0",

    "flask==3.0.3",
    "flask-socketio>=5.5.1,<6.0.0",
    "matplotlib>=3.7.5",

    "seaborn",
    "rerun-sdk (==0.22.0)",
    "robotic-arm (>=1.0.6,<2.0.0)",
    "websockets",

    "colorama",
    "scipy",
    "logging_mp"
```
