#!/usr/bin/env bash

# Read a single KEY=value from docker/.env without sourcing arbitrary shell.
deployment_env_value() {
  local env_file=$1
  local key=$2
  [[ -f "$env_file" ]] || return 0
  awk -F= -v key="$key" '
    $1 == key {
      sub(/^[^=]*=/, "")
      sub(/\r$/, "")
      print
      exit
    }
  ' "$env_file"
}

deployment_env_default() {
  local env_file=$1
  local key=$2
  local fallback=$3
  local value
  value="$(deployment_env_value "$env_file" "$key")"
  printf '%s\n' "${value:-$fallback}"
}
