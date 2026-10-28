#!/usr/bin/env bash
# Manually convert one finalized source bag into an atomically published dataset.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if (($# < 2)); then
  echo "Usage: $0 SOURCE_BAG OUTPUT_DATASET [--segments all|full|milestones] [--task DESCRIPTION]" >&2
  exit 2
fi

SOURCE_BAG="$(realpath -- "$1")"
OUTPUT_DATASET="$(realpath -m -- "$2")"
shift 2
CONVERSION_OPTIONS=()
while (($#)); do
  case "$1" in
    --segments|--task)
      (($# >= 2)) || { echo "Missing value for $1" >&2; exit 2; }
      CONVERSION_OPTIONS+=("$1" "$2")
      shift 2
      ;;
    *)
      echo "Unknown conversion option: $1" >&2
      exit 2
      ;;
  esac
done
[[ -f "$SOURCE_BAG/metadata.yaml" ]] || {
  echo "Source is not a ROS bag directory: $SOURCE_BAG" >&2
  exit 2
}
[[ -f "$SOURCE_BAG/collection_state.json" ]] || {
  echo "Source bag is missing collection_state.json: $SOURCE_BAG" >&2
  exit 2
}
if [[ "$OUTPUT_DATASET" == "$SOURCE_BAG" || "$OUTPUT_DATASET/" == "$SOURCE_BAG/"* ]]; then
  echo "Output dataset must not be the source bag or one of its children" >&2
  exit 2
fi
if [[ -e "$OUTPUT_DATASET" ]]; then
  echo "Output already exists; refusing to overwrite: $OUTPUT_DATASET" >&2
  exit 2
fi

OUTPUT_PARENT="$(dirname -- "$OUTPUT_DATASET")"
OUTPUT_NAME="$(basename -- "$OUTPUT_DATASET")"
mkdir -p -- "$OUTPUT_PARENT"

(
  cd "$REPO_ROOT/docker"
  docker compose run --rm --no-TTY \
    -v "$SOURCE_BAG:/source_bag:ro" \
    -v "$OUTPUT_PARENT:/dataset_parent" \
    tools \
    ros2 run teleop_data_collector rosbag_to_lerobot \
      --config /workspace/franka_upper_body_teleop/data_collection/config/convert_gello_lerobot_v2.yaml \
      /source_bag --output "/dataset_parent/$OUTPUT_NAME" "${CONVERSION_OPTIONS[@]}"
)
