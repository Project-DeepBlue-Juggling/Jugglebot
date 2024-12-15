#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

# TASK [Define functions]

task() {
  local task_desc="$1"
  echo -e "\nTASK [${task_desc}] ********"
}

does_docker_volume_exist() {
  local volume_name="$1"
  [[ -n "$(docker volume ls --quiet --filter "name=${volume_name}")" ]]
}

is_docker_container_running() {
  local container_name="$1"
  [[ "$(docker container ls --quiet --filter "name=${container_name}")" != '' ]]
}

does_docker_container_exist() {
  local container_name="$1"
  [[ "$(docker container ls --quiet --all --filter "name=${container_name}")" != '' ]]
}

is_home_dir_initialized() {
  local container_name="$1"
  docker exec -it \
    "${container_name}" \
    /usr/bin/bash -c '[[ -f "${HOME}/.user-dir-initialized" ]]'
}

task 'Parse the arguments'

while [[ $# -gt 0 ]]; do
  case $1 in
    -k|--ssh-keypair-name)
      SSH_KEYPAIR_NAME="$2"
      shift
      shift
      ;;
    -e|--debug-environments-dir)
      ENVIRONMENTS_DIR="$2"
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

if [[ -z "${ENVIRONMENTS_DIR:-}" ]]; then
  ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--debug-environments-dir' flag should only be used when testing this script.\n"
fi

DEV_ENV_USERNAME='devops' # This is also the default password for the user.
SSH_PRIVATE_KEY_FILEPATH="${HOME}/.ssh/${SSH_KEYPAIR_NAME}"
BUILD_CONTEXT_DIR="${ENVIRONMENTS_DIR}/ubuntu_20.04-docker-native"
IMAGE_NAME='jugglebot-native-dev:focal'
CONTAINER_NAME='jugglebot-native-dev'
HOME_VOLUME_NAME='jugglebot-native-dev-home'

task 'Enable ssh-agent'

eval "$(ssh-agent -s)"

task 'Add the ssh private key'

# Note: This will prompt for the passphrase if the key requires one 

ssh-add "${SSH_PRIVATE_KEY_FILEPATH}"

task 'Copy ~/.gitconfig into the build context'

install -D -T "${HOME}/.gitconfig" "${BUILD_CONTEXT_DIR}/build/gitconfig"

task "Build the docker image named ${IMAGE_NAME}"

docker buildx build \
  --build-arg "USER_UID=$(id --user)" \
  --build-arg "USER_GID=$(id --group)" \
  --build-arg "DOCKER_GID=$(stat -c '%g' /var/run/docker.sock)" \
  --build-arg "USERNAME=${DEV_ENV_USERNAME}" \
  --ssh "default=${SSH_AUTH_SOCK}" \
  --progress=tty \
  -t "${IMAGE_NAME}" \
  "${BUILD_CONTEXT_DIR}"

task 'Cleanup the build context'

rm "${BUILD_CONTEXT_DIR}/build/gitconfig"

task "Ensure that the docker volume named ${HOME_VOLUME_NAME} exists"

if ! does_docker_volume_exist "${HOME_VOLUME_NAME}"; then
  docker volume create "${HOME_VOLUME_NAME}"  
fi

task 'Ensure that the docker container is not running'

if is_docker_container_running "${CONTAINER_NAME}"; then
  docker container stop "${CONTAINER_NAME}"
fi

task "Ensure that the ${CONTAINER_NAME} docker container does not exist"

if does_docker_container_exist "${CONTAINER_NAME}"; then
  docker container rm --force --volumes "${CONTAINER_NAME}"
fi

task "Create the ${CONTAINER_NAME} docker container"

# Note: We expose /tmp because that is where $SSH_AUTH_SOCK is stored. We
# expose /var/run/docker.sock because that enables the container to control the
# shared Docker Engine. We expose ~/.oh-my-zsh/custom so that we can more
# easily keep aliases and shell features in sync across environments.

docker container create --name "${CONTAINER_NAME}" \
  -v '/tmp:/tmp' \
  -v '/var/run/docker.sock:/var/run/docker.sock' \
  -v "${HOME_VOLUME_NAME}:/home" \
  -v "${HOME}/.oh-my-zsh/custom:/entrypoint/oh-my-zsh-custom" \
  --dns '8.8.8.8' \
  "${IMAGE_NAME}"

task 'Start the container to begin initializing the home directory'

docker container start "${CONTAINER_NAME}"

task 'Wait for the home directory to be initialized by entrypoint.sh'

while ! is_home_dir_initialized "${CONTAINER_NAME}"; do
  echo "Waiting for the container ${CONTAINER_NAME} to be initialized..."
  sleep 2
done

task 'Stop the container'

docker container stop "${CONTAINER_NAME}"

task 'Prompt next steps'

echo -e "
Run the following command to open a shell in the jugglebot-native-dev
container:

docker-native-env

---

Notes:

1. By default, the user in the container is named ${DEV_ENV_USERNAME}, and the password for
   that user is the same as the username.

2. Only the /home directory and the /tmp directory will persist across 
   container restarts. The /home directory is a mounted docker volume named 
   ${HOME_VOLUME_NAME}.

3. If you add your GitHub ssh key to the keychain prior to running
   docker-native-env, that key will be available within the container. This
   works because (a) the container shares the same /tmp directory with the
   WSL2 environment and (b) the docker-native-env script propagates the 
   SSH_AUTH_SOCK environment variable into the container. You can add the key
   to the keychain using the following command:

   ssh-add "${SSH_PRIVATE_KEY_FILEPATH}"

4. You can control the Docker Engine that is hosted by the WSL2 environment 
   from within the container. This has some security implications. Do not host
   internet-facing services within the container.

5. The ~/.oh-my-zsh/custom directory in the WSL2 environment is mounted to the 
   /home/${DEV_ENV_USERNAME}/.oh-my-zsh/custom directory in the container. This simplifies
   keeping the environments and aliases in sync.
"

