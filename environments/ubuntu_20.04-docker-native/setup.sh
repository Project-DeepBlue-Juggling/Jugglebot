#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

task() {
  local task_desc="$1"
  echo -e "\nTASK [${task_desc}] ********"
}

task 'Begin building the base_setup.sh command'

GUEST_ENVIRONMENTS_DIR='/home/devops/Jugglebot/environments'

BASE_SETUP_COMMAND_PARTS=()

BASE_SETUP_COMMAND_PARTS+=("${GUEST_ENVIRONMENTS_DIR}/ubuntu-common/base_setup.sh")

task 'Parse the arguments while coddling the unrecognized arguments'

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--debug-environments-dir)
      HOST_ENVIRONMENTS_DIR="$2"
      shift
      shift
      ;;
    -*|--*)
      # Note: Each of the base_setup.sh flags has a parameter.
      BASE_SETUP_COMMAND_PARTS+=("$1")
      BASE_SETUP_COMMAND_PARTS+=("'$2'")
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

if [[ -z "${HOST_ENVIRONMENTS_DIR:-}" ]]; then
  HOST_ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--debug-environments-dir' flag should only be used when testing this script.\n"
fi

task 'Prepare additional arguments for ubuntu-common/base_setup.sh'

BASE_SETUP_COMMAND_PARTS+=('--jugglebot-conda-env-filepath')
BASE_SETUP_COMMAND_PARTS+=("'${GUEST_ENVIRONMENTS_DIR}/ubuntu-common/jugglebot_conda_env.yml'")
BASE_SETUP_COMMAND_PARTS+=('--ansible-playbook-filepath')
BASE_SETUP_COMMAND_PARTS+=("'${GUEST_ENVIRONMENTS_DIR}/ubuntu_20.04-docker-native/main_playbook.yml'")

task 'Build the docker image named jugglebot-dev:focal-native'

docker build -t jugglebot-dev:focal-native "${HOST_ENVIRONMENTS_DIR}/ubuntu_20.04-docker-native"

task 'Invoke ubuntu-common/base_setup.sh while including the coddled arguments'

BASE_SETUP_COMMAND="${BASE_SETUP_COMMAND_PARTS[*]}"

docker run \
  -v ~/docker_native_home:/home/devops \
  -v ~/.ssh:/home/devops/.ssh \
  -v /tmp/.ssh:/tmp/.ssh \
  -v ~/Jugglebot:/home/devops/Jugglebot \
  -v ~/.oh-my-zsh/custom:/home/devops/.oh-my-zsh-custom \
  -e SSH_AUTH_SOCK=/tmp/.ssh/ssh_auth_sock \
  --dns 8.8.8.8 \
  -it jugglebot-dev:focal-native \
  /bin/bash -c "${BASE_SETUP_COMMAND}"

