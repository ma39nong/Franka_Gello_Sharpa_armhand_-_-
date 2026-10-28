#!/usr/bin/env bash
# Revalidate a completed source bag and atomically update only its state sidecar.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if (($# < 1)); then
  echo "Usage: $0 SOURCE_BAG [--trim-start-sec SEC] [--trim-end-sec SEC]" >&2
  exit 2
fi

SOURCE_BAG="$(realpath -- "$1")"
shift
[[ -f "$SOURCE_BAG/metadata.yaml" ]] || {
  echo "Source is not a ROS bag directory: $SOURCE_BAG" >&2
  exit 2
}
[[ -f "$SOURCE_BAG/collection_state.json" ]] || {
  echo "Source bag is missing collection_state.json: $SOURCE_BAG" >&2
  exit 2
}

(
  cd "$REPO_ROOT/docker"
  docker compose run --rm --no-TTY \
    -v "$SOURCE_BAG:/source_bag" \
    tools \
    python3 -m teleop_data_collector.revalidate_bag /source_bag "$@"
)
