#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

task() {
  local task_desc="$1"
  echo -e "\nTASK [${task_desc}] ********"
}


task 'Parse the arguments'

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--debug-environments-dir)
      HOST_ENVIRONMENTS_DIR="$2"
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

task 'Assert that an ssh keypair name was specified'

if [[ -z "${SSH_KEYPAIR_NAME:-}" ]]; then
  echo '[ERROR]: An ssh keypair name is required. Invoke this command with the `--ssh-keypair-name [keypair name]` switch (eg. `--ssh-keypair-name ed25519`)'
  exit 2
fi

task 'Initialize variables'

if [[ -z "${HOST_ENVIRONMENTS_DIR:-}" ]]; then
  HOST_ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--debug-environments-dir' flag should only be used when testing this script.\n"
fi

GUEST_ENVIRONMENTS_DIR='/home/devops/Jugglebot/environments'
SSH_PRIVATE_KEY_FILEPATH="${HOME}/.ssh/${SSH_KEYPAIR_NAME}"

task 'Enable ssh-agent'

eval "$(ssh-agent -s)"

task 'Add the ssh private key'

# Note: This will prompt for the passphrase if the key requires one 

ssh-add "${SSH_PRIVATE_KEY_FILEPATH}"

task 'Build the docker image named jugglebot-dev:focal-native'

docker build -t jugglebot-dev:focal-native "${HOST_ENVIRONMENTS_DIR}/ubuntu_20.04-docker-native"

ln -s -f -T "${HOST_ENVIRONMENTS_DIR}/ubuntu_20.04-docker-native/docker-native-env" "${HOME}/bin/docker-native-env"

docker-native-env

