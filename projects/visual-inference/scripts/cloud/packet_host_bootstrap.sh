#!/bin/sh
set -eu
bootstrap_dir=/var/lib/visual-inference-bootstrap
bootstrap_status=$bootstrap_dir/status
bootstrap_log=/var/log/visual-inference-bootstrap.log
install -d -m 755 "$bootstrap_dir"
: > "$bootstrap_log"
exec 3>&1
exec >> "$bootstrap_log" 2>&1
progress() {
  printf '%s\n' "$1"
  printf '%s\n' "$1" >&3
}
printf '%s\n' running > "$bootstrap_status"
progress 'Packet bootstrap: initializing SSH access'
bootstrap_exit() {
  code=$?
  if [ "$code" -ne 0 ]; then
    printf 'failed:%s\n' "$code" > "$bootstrap_status"
    progress "Packet bootstrap: failed with exit code $code"
  fi
}
trap bootstrap_exit EXIT

key=$(printf '%s' '__ENCODED_PUBLIC_KEY__' | base64 -d)
install_key() {
  user=$1
  home=$2
  install -d -m 700 -o "$user" -g "$user" "$home/.ssh"
  touch "$home/.ssh/authorized_keys"
  grep -qxF "$key" "$home/.ssh/authorized_keys" || printf '%s\n' "$key" >> "$home/.ssh/authorized_keys"
  chown "$user:$user" "$home/.ssh/authorized_keys"
  chmod 600 "$home/.ssh/authorized_keys"
}
install_key root /root
if id ubuntu >/dev/null 2>&1; then
  install_key ubuntu /home/ubuntu
  if [ -d /etc/sudoers.d ]; then
    printf '%s\n' 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-dstack
    chmod 440 /etc/sudoers.d/90-dstack
  fi
fi
if [ -d /etc/ssh/sshd_config.d ]; then
  printf '%s\n' 'AllowTcpForwarding yes' > /etc/ssh/sshd_config.d/90-dstack.conf
fi
systemctl reload ssh 2>/dev/null || systemctl reload sshd 2>/dev/null || true
progress 'Packet bootstrap: SSH access configured'

retry() {
  attempts=0
  until "$@"; do
    attempts=$((attempts + 1))
    if [ "$attempts" -ge 5 ]; then
      return 1
    fi
    progress "Packet bootstrap: command failed; retrying in $((attempts * 5))s: $*"
    sleep $((attempts * 5))
  done
}

if [ ! -r /etc/os-release ]; then
  echo "Packet host has no /etc/os-release"
  exit 30
fi
. /etc/os-release
if [ "$ID" != ubuntu ]; then
  echo "Packet host OS '$ID' is unsupported; expected Ubuntu"
  exit 31
fi
case "$VERSION_ID" in
  22.04|24.04|25.10|26.04) ;;
  *)
    echo "Packet host Ubuntu $VERSION_ID is unsupported by this Docker release"
    exit 31
    ;;
esac

export DEBIAN_FRONTEND=noninteractive
progress 'Packet bootstrap: installing package-manager prerequisites'
retry apt-get update
retry apt-get install -y ca-certificates curl gnupg jq
install -m 0755 -d /etc/apt/keyrings

conflicting_packages=$(dpkg-query -W -f='${binary:Package} ${db:Status-Abbrev}\n' \
  docker.io docker-compose docker-compose-v2 docker-doc podman-docker \
  containerd runc 2>/dev/null | awk '$2 ~ /^ii/ {print $1}')
if [ -n "$conflicting_packages" ]; then
  # These distro packages conflict with Docker CE's bundled containerd/runc.
  retry apt-get remove -y $conflicting_packages
fi

progress 'Packet bootstrap: configuring the Docker repository'
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
docker_codename=${UBUNTU_CODENAME:-$VERSION_CODENAME}
docker_arch=$(dpkg --print-architecture)
printf '%s\n' \
  'Types: deb' \
  'URIs: https://download.docker.com/linux/ubuntu' \
  "Suites: $docker_codename" \
  'Components: stable' \
  "Architectures: $docker_arch" \
  'Signed-By: /etc/apt/keyrings/docker.asc' \
  > /etc/apt/sources.list.d/docker.sources

progress 'Packet bootstrap: configuring the NVIDIA repository'
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  -o /tmp/nvidia-container-toolkit.gpgkey
gpg --dearmor --yes \
  -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  /tmp/nvidia-container-toolkit.gpgkey
curl -fsSL \
  https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  -o /tmp/nvidia-container-toolkit.list
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  /tmp/nvidia-container-toolkit.list \
  > /etc/apt/sources.list.d/nvidia-container-toolkit.list

progress 'Packet bootstrap: installing Docker Engine'
retry apt-get update
docker_version=$(apt-cache madison docker-ce | awk -v wanted="__DOCKER_VERSION__" \
  '$3 ~ (":" wanted "-") {print $3; exit}')
if [ -z "$docker_version" ]; then
  echo 'Pinned Docker Engine __DOCKER_VERSION__ is unavailable for this Ubuntu image'
  exit 32
fi
retry apt-get install -y \
  "docker-ce=$docker_version" \
  "docker-ce-cli=$docker_version" \
  containerd.io
progress 'Packet bootstrap: installing NVIDIA Container Toolkit'
retry apt-get install -y \
  nvidia-container-toolkit=__NVIDIA_TOOLKIT_VERSION__ \
  nvidia-container-toolkit-base=__NVIDIA_TOOLKIT_VERSION__ \
  libnvidia-container-tools=__NVIDIA_TOOLKIT_VERSION__ \
  libnvidia-container1=__NVIDIA_TOOLKIT_VERSION__

progress 'Packet bootstrap: configuring Docker runtime defaults'
nvidia-ctk runtime configure --runtime=docker
daemon_config=/etc/docker/daemon.json
daemon_config_tmp=$(mktemp)
jq '. + {"default-shm-size": "32G"}' "$daemon_config" > "$daemon_config_tmp"
install -m 0644 "$daemon_config_tmp" "$daemon_config"
rm -f "$daemon_config_tmp"
dockerd --validate --config-file "$daemon_config"

progress 'Packet bootstrap: verifying the NVIDIA Docker runtime'
systemctl enable --now docker
systemctl restart docker
docker info >/dev/null
[ "$(jq -r '.runtimes.nvidia.path // empty' "$daemon_config")" != "" ] || {
  echo 'Docker daemon configuration lost the NVIDIA runtime'
  exit 33
}
[ "$(jq -r '.["default-shm-size"] // empty' "$daemon_config")" = 32G ] || {
  echo 'Docker daemon configuration lost the 32G shared-memory default'
  exit 33
}
nvidia-smi -L
nvidia-container-cli info >/dev/null
docker info --format '{{json .Runtimes}}' | grep -qi nvidia

printf '%s\n' ready > "$bootstrap_status"
trap - EXIT
progress 'Packet bootstrap: completed successfully'
