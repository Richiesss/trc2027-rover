# trc2027-rover 本番環境（ROS 2）
#
# シミュレーション環境（trc2027-sim）と同じ ROS 2 Jazzy を、Jetson（JetPack 6 / Ubuntu 22.04）でも
# 動かすためのイメージ。公式の ros イメージは arm64 / amd64 の両方に対応しているので、
# Jetson でも開発用PCでも同じ Dockerfile でビルドできる。
#
# 注意：このイメージでは GPU（CUDA）と CSIカメラ（nvargus）は使えない。
#       必要になったら jetson-containers（dustynv/ros 系）のイメージに FROM を差し替える。
#       詳しくは docs/docker.md を参照。

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# 時間のかかる apt は先に済ませておき、キャッシュを効かせる
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-serial \
        python3-numpy \
        python3-opencv \
        ros-${ROS_DISTRO}-cv-bridge \
        usbutils \
        iproute2 \
        iputils-ping \
        less \
        nano \
    && rm -rf /var/lib/apt/lists/*

# apt にない Python ライブラリ（バージョンを固定）
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

WORKDIR /ws

# コンテナに入ったら ROS 2 と、ビルド済みのワークスペースを自動で読み込む
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

# 開発中は起動したまま待機させ、docker compose exec で中に入る
CMD ["sleep", "infinity"]
