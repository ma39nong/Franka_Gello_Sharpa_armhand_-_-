#!/usr/bin/env bash
# Build only the Orbbec camera runtime. Does not open devices or start services.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repo/ops/lib/deployment_env.sh"
accepted="${ORBBEC_SDK_LICENSE_ACCEPTED:-$(deployment_env_value "$repo/docker/.env" ORBBEC_SDK_LICENSE_ACCEPTED)}"
[[ "$accepted" == YES ]] || { echo 'Review and accept the Orbbec SDK license in docker/.env first.' >&2; exit 1; }
archive="$(realpath -- "${1:?Usage: build_camera_sdk.sh /path/to/official-SDK-2.10.6.tar.gz}")"
expected=23ec9f7023881cc9061bf0b4ba65735fb16c10e168fd5d62b1548c847e7ddb46
actual="$(sha256sum -- "$archive")"
[[ "${actual%% *}" == "$expected" ]] || { echo 'Official SDK archive SHA-256 mismatch' >&2; exit 1; }
context="$(mktemp -d)"
trap 'rm -rf -- "$context"' EXIT
mkdir "$context/sdk"
tar -xzf "$archive" --strip-components=1 -C "$context/sdk"
cp "$repo/docker/camera_sdk_entrypoint.sh" "$context/"
docker build --network=none -f "$repo/docker/camera-sdk.Dockerfile" \
  -t gello-teleop-cameras:orbbec-sdk-2.10.6 "$context"
