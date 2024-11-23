#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

task() {
  local name="$1"
  echo -e "\nTASK [${name}] ********"
}

task 'Parse the command line arguments'

while [[ $# -gt 0 ]]; do
  case $1 in
    -k|--ssh-keypair-name)
      SSH_KEYPAIR_NAME="$2"
      shift
      shift
      ;;
    -e|--jugglebot-environments-dir)
      JUGGLEBOT_ENVIRONMENTS_DIR="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "[ERROR]: Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift # past argument
      ;;
  esac
done

task 'Assert that an ssh keypair name was specified'

if [[ -z "${SSH_KEYPAIR_NAME:-}" ]]; then
  echo '[ERROR]: An ssh keypair name is required. Invoke this command with the `--ssh-keypair-name [keypair name]` switch (eg. `--ssh-keypair-name ed25519`)'
  exit 2
fi

task 'Warn that setting an alternative environments directory is not supported.'

if [[ -z "${JUGGLEBOT_ENVIRONMENTS_DIR:-}" ]]; then
  JUGGLEBOT_ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--jugglebot-environments-dir' flag should only be used when testing this script.\n"
fi

task 'Initialize variables'

CONDA_SETUP_SCRIPT_URL='https://github.com/conda-forge/miniforge/releases/download/24.9.0-0/Miniforge3-24.9.0-0-Linux-x86_64.sh'

HOST_SETUP_DIR="${HOME}/.jugglebot/host_setup"
HOST_SETUP_BACKUPS_DIR="${HOME}/.jugglebot/host_setup/backups"
CONDA_SETUP_SCRIPT_FILEPATH="${HOST_SETUP_DIR}/miniforge_setup.sh"
CONDA_FILEPATH="${HOME}/miniforge3/bin/conda"
JUGGLEBOT_CONDA_ENV_FILEPATH="${JUGGLEBOT_ENVIRONMENTS_DIR}/ubuntu-common/jugglebot_conda_env.yml"
WSL2_MAIN_PLAYBOOK_FILEPATH="${JUGGLEBOT_ENVIRONMENTS_DIR}/ubuntu_24.04-wsl2/main_playbook.yml"
SSH_PRIVATE_KEY_FILEPATH="${HOME}/.ssh/${SSH_KEYPAIR_NAME}"

task 'Ensure that the ~/.jugglebot/host_setup/backups directory exists'

install -d "${HOST_SETUP_BACKUPS_DIR}"

task 'Download the conda setup script'

# when: the setup script doesn't exist

if [[ ! -f "${CONDA_SETUP_SCRIPT_FILEPATH}" ]]; then
  wget "${CONDA_SETUP_SCRIPT_URL}" -O - > "${CONDA_SETUP_SCRIPT_FILEPATH}" || rc="$?"
fi

# failed_when: the return code is nonzero

if [[ $rc -ne 0 ]]; then
  echo "[ERROR]: The Conda setup script could not be downloaded from `${CONDA_SETUP_SCRIPT_URL}`."
  exit $rc
fi

task 'Run the conda setup script'

if [[ ! -f "${CONDA_FILEPATH}" ]]; then
  # The `-b` flag signifies batch mode, which applies the default config.
  sh "${CONDA_SETUP_SCRIPT_FILEPATH}" -b 
fi

# failed_when: the setup script didn't produce the conda executable

if [[ ! -f "${CONDA_FILEPATH}" ]]; then
  echo "[ERROR]: The Conda setup script did not provision `${CONDA_FILEPATH}`."
  exit 3
fi

task 'Install conda in the bashrc'

"${CONDA_FILEPATH}" init bash || true

task 'Enable conda'

eval "$("${CONDA_FILEPATH}" 'shell.bash' 'hook' 2> /dev/null)"

task 'Update conda base'

conda update -y -n base -c conda-forge conda

task 'Determine whether the jugglebot conda environment exists'

JUGGLEBOT_CONDA_ENV_STATE="$( if conda info --envs | grep -q '^jugglebot\s'; then echo 'present'; else echo 'absent'; fi )"

if [[ "${JUGGLEBOT_CONDA_ENV_STATE}" == 'absent' ]]; then

  task 'Create the jugglebot conda environment'

  conda env create -f "${JUGGLEBOT_CONDA_ENV_FILEPATH}"

else
  
  JUGGLEBOT_CONDA_ENV_BACKUP_FILEPATH="$( mktemp -p "${HOST_SETUP_BACKUPS_DIR}" "jugglebot_conda_env.$( date +'%Y-%m-%dT%H-%M-%S%z' )_XXXXXXXX.yml" )"
  
  task "Backup the jugglebot conda environment to ${JUGGLEBOT_CONDA_ENV_BACKUP_FILEPATH}"

  conda env export --from-history > "${JUGGLEBOT_CONDA_ENV_BACKUP_FILEPATH}"

  task 'Update the jugglebot conda environment'

  conda env update -f "${JUGGLEBOT_CONDA_ENV_FILEPATH}" --prune

fi

task 'Activate the jugglebot conda environment'

conda activate jugglebot

task 'Disable shell prompt modification by conda'

conda config --set changeps1 False

task 'Enable ssh-agent'

eval "$(ssh-agent -s)"

task 'Add the ssh private key'

# Note: This will prompt for the passphrase if the key requires one 

ssh-add "${SSH_PRIVATE_KEY_FILEPATH}"

task 'Run the WSL2 main playbook'

echo -e "\nEnter your password to enable ${WSL2_MAIN_PLAYBOOK_FILEPATH} to perform privileged operations"
ANSIBLE_LOCALHOST_WARNING=False ANSIBLE_INVENTORY_UNPARSED_WARNING=False ansible-playbook "${WSL2_MAIN_PLAYBOOK_FILEPATH}" --ask-become-pass -e 'upgrade_software=yes' -e "ssh_keypair_name='${SSH_KEYPAIR_NAME}'" || rc="$?"

# failed_when: the return code is nonzero

if [[ $rc -ne 0 ]]; then
  echo -e "[ERROR]: The ${WSL2_MAIN_PLAYBOOK_FILEPATH} failed with return code ${rc}. After diagnosing the cause of the failure, you can re-run it using the following command:\n\nANSIBLE_LOCALHOST_WARNING=False ANSIBLE_INVENTORY_UNPARSED_WARNING=False ansible-playbook '${WSL2_MAIN_PLAYBOOK_FILEPATH}' --ask-become-pass -e 'upgrade_software=yes' -e 'ssh_keypair_name=${SSH_KEYPAIR_NAME}'"
  exit $rc
fi

