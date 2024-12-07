#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

task() {
  local task_desc="$1"
  echo -e "\nTASK [${task_desc}] ********"
}

task 'Parse the arguments while coddling the unrecognized arguments'

BASE_SETUP_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--debug-environments-dir)
      ENVIRONMENTS_DIR="$2"
      shift
      shift
      ;;
    -*|--*)
      # Note: Each of the base_setup.sh flags has a parameter.
      BASE_SETUP_ARGS+=("$1")
      BASE_SETUP_ARGS+=("$2")
      shift
      shift
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift
      ;;
  esac
done

task 'Initialize variables'

if [[ -z "${ENVIRONMENTS_DIR:-}" ]]; then
  ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--debug-environments-dir' flag should only be used when testing this script.\n"
fi

JUGGLEBOT_CONDA_ENV_FILEPATH="${ENVIRONMENTS_DIR}/ubuntu-common/jugglebot_conda_env.yml"
ANSIBLE_PLAYBOOK_FILEPATH="${ENVIRONMENTS_DIR}/ubuntu_24.04-wsl2/main_playbook.yml"

task 'Prepare the arguments for ubuntu-common/base_setup.sh'

BASE_SETUP_ARGS+=('--jugglebot-conda-env-filepath')
BASE_SETUP_ARGS+=("${JUGGLEBOT_CONDA_ENV_FILEPATH}")
BASE_SETUP_ARGS+=('--ansible-playbook-filepath')
BASE_SETUP_ARGS+=("${ANSIBLE_PLAYBOOK_FILEPATH}")

task 'Invoke ubuntu-common/base_setup.sh while including the coddled arguments'

${ENVIRONMENTS_DIR}/ubuntu-common/base_setup.sh "${BASE_SETUP_ARGS[@]}"

