# platform_docker

技術実証プラットフォーム向けDocker関連ファイル．
例えばtemplateからDockerイメージを生成する場合は以下のコマンドを入力する．

```bash
cd <path_to_int-ball2_platform_works>/platform_docker/template
docker build . -t ib2_user:0.1
```

## demos
Dockerコンテナ内でユーザプログラムを動作させるデモプログラム．

### ros1_melodic:
#### Environments
- Container image: Ubuntu 20.04 LTS
- ROS1: Melodic

`simple_talker`ノード (ROS Melodic) を技術実証プラットフォーム用GSEから起動するデモプログラム．

### ros2_foxy
#### Environments
- Container image: ros:melodic (Ubuntu 18.04 LTS)
- ROS1: Noetic
- ROS2: Foxy

`simple_talker`ノード (ROS Noetic) と`simple_listener`ノード (ROS Foxy) を技術実証プラットフォーム用GSEから起動するデモプログラム．
`ros1_bridge`を介して相互通信させる．

### ros2_humble
#### Environments
- Container image: Ubuntu 22.04 LTS
- ROS1: Noetic 
- ROS2: Humble

`simple_talker`ノード (ROS Noetic) と`simple_listener`ノード (ROS Humble) を技術実証プラットフォーム用GSEから起動するデモプログラム．
`ros1_bridge`を介して相互通信させる．

## template (**非推奨**)
最小限のテンプレートパッケージ．
ユーザプログラムは技術実証プラットフォーム上に配置する想定．
