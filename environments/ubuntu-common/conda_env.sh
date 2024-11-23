#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--event-type)
      EVENT_TYPE="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "[ERROR]: Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${EVENT_TYPE}" == "activate" ]]; then
 
  
  # TASK [Activate] ********

  export JUGGLEBOT_REPO_DIR="${HOME}/Jugglebot"
  export JUGGLEBOT_CONFIG_DIR="${HOME}/.jugglebot"


elif [[ "${EVENT_TYPE}" == "deactivate" ]]; then

  
  # TASK [Deactivate] ********

  unset JUGGLEBOT_REPO_DIR
  unset JUGGLEBOT_CONFIG_DIR


else
  echo -e "\n[ERROR]: Unrecognized event type (${EVENT_TYPE})."
  exit 2
fi

