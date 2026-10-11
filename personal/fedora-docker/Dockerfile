# Personal Fedora/Ubuntu bridge image for MAAV nav onboarding.
# NOT team infrastructure — do not merge into main/dev-test.
#
# Base: official PX4 ROS 2 Humble toolchain + Gazebo Harmonic + Python deps for PX4 SITL.
FROM px4io/px4-dev-ros2:main-humble

ENV DEBIAN_FRONTEND=noninteractive

USER root
RUN printf 'nameserver 8.8.8.8\nnameserver 1.1.1.1\n' > /etc/resolv.conf \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      curl ca-certificates gnupg lsb-release dbus \
      libopencv-dev \
      python3-pip \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends gz-harmonic \
 && pip3 install --no-cache-dir \
      kconfiglib jinja2 pyros-genmsg packaging jsonschema pyyaml future \
 && rm -rf /var/lib/apt/lists/*

# Match a normal host user (uid 1000) for bind-mounted $HOME
RUN groupadd -g 1000 hostuser 2>/dev/null || true \
 && useradd -u 1000 -g 1000 -d /home/hostuser -s /bin/bash -M hostuser 2>/dev/null || true

CMD ["sleep", "infinity"]
