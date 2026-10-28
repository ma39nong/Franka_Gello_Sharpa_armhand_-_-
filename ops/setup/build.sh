#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
image=franka-upper-body-teleop:latest
accept_orbbec=false
reuse_image=false
rebuild_image=false
build_args=()
for argument in "$@"; do
  case "$argument" in
    --accept-orbbec-eula) accept_orbbec=true ;;
    --reuse-image) reuse_image=true ;;
    --rebuild-image) rebuild_image=true ;;
    *) build_args+=("$argument") ;;
  esac
done

if [[ "$reuse_image" == true && "$rebuild_image" == true ]]; then
  echo "Use only one of --reuse-image or --rebuild-image." >&2
  exit 2
fi

# BuildKit injects the daemon HTTP proxy into RUN. Clash/mihomo on
# 127.0.0.1:7897 is the host listener; inside a build container that address
# is empty, so apt fails with "Connection refused". Host networking makes
# the loopback proxy reachable.
proxy_url="${HTTP_PROXY:-${http_proxy:-}}"
if [[ -z "$proxy_url" ]]; then
  proxy_url="$(docker info --format '{{.HTTPProxy}}' 2>/dev/null || true)"
fi
docker_build_opts=()
if [[ "$proxy_url" == *127.0.0.1* || "$proxy_url" == *localhost* ]]; then
  echo "Local HTTP proxy ${proxy_url} is not reachable from a build container; using --network=host."
  docker_build_opts+=(--network=host)
fi

image_has_collection_python() {
  docker run --rm --network none "$image" \
    python3 -c "import pyarrow, cv2" >/dev/null 2>&1
}

layer_collection_python() {
  echo "Adding python3-opencv and pyarrow onto existing ${image}."
  docker build "${docker_build_opts[@]}" --tag "$image" - <<'EOF'
FROM franka-upper-body-teleop:latest
RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv \
    && python3 -m pip install --no-cache-dir 'pyarrow==17.0.0' \
    && rm -rf /var/lib/apt/lists/*
EOF
}

if [[ "$reuse_image" == true ]]; then
  if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "No ${image} to reuse; build without --reuse-image or load the workstation image." >&2
    exit 1
  fi
  if image_has_collection_python; then
    echo "Reusing ${image}; python3-opencv and pyarrow are already present."
  else
    layer_collection_python
  fi
elif [[ "$rebuild_image" == true ]] || ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build "${docker_build_opts[@]}" --tag "$image" "${repo}/docker"
else
  # An existing workstation image is not a BuildKit layer cache. Re-running
  # the full Dockerfile would rebuild from the ROS apt layer (hours) even
  # when only collection Python deps changed. Reuse unless forced.
  if image_has_collection_python; then
    echo "Reusing ${image}; python3-opencv and pyarrow are already present."
    echo "Pass --rebuild-image to build the Dockerfile from the ROS base."
  else
    layer_collection_python
  fi
fi

orbbec_ignore=(
  "$repo/ros_ws/src/orbbec_camera/COLCON_IGNORE"
  "$repo/ros_ws/src/orbbec_camera_msgs/COLCON_IGNORE"
)
docker_environment=()
if [[ "$accept_orbbec" == true ]]; then
  docker_environment=(--env LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME=ON)
  echo "Building the bundled Orbbec runtime after explicit --accept-orbbec-eula."
  rm -f -- "${orbbec_ignore[@]}"
else
  # Keep the Harvest Orbbec packages from overlaying the vendor package used by
  # compose service `orbbec` during a normal teleop workspace build.
  mkdir -p -- "$repo/ros_ws/src/orbbec_camera" "$repo/ros_ws/src/orbbec_camera_msgs"
  touch -- "${orbbec_ignore[@]}"
fi

docker run --rm \
  "${docker_environment[@]}" \
  --volume "${repo}:/workspace/franka_upper_body_teleop" \
  --workdir /workspace/franka_upper_body_teleop \
  "$image" \
  docker/build_workspace.sh "${build_args[@]}"

# Restore ignore files so a later default build cannot replace vendor Orbbec
# with the license-gated stub package.
touch -- "${orbbec_ignore[@]}"
