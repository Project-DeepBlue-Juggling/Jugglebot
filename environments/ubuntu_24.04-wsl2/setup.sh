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
    -k|--ssh-keypair-name)
      SSH_KEYPAIR_NAME="$2"
      shift
      shift
      ;;
    -N|--git-name)
      GIT_NAME="$2"
      shift
      shift
      ;;
    -E|--git-email)
      GIT_EMAIL="$2"
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

task 'Assert that a git name was specified'

if [[ -z "${GIT_NAME:-}" ]]; then
  echo '[ERROR]: A git name is required. Invoke this command with the `--git-name "[Your full name]"` switch (eg. `--git-name "Jane Doe"`)'
  exit 2
fi

task 'Assert that a git email was specified'

if [[ -z "${GIT_EMAIL:-}" ]]; then
  echo '[ERROR]: A git email is required. Invoke this command with the `--git-email "[your email address]"` switch (eg. `--git-email "jane.doe@gmail.com"`)'
  exit 2
fi

task 'Initialize variables'

if [[ -z "${ENVIRONMENTS_DIR:-}" ]]; then
  ENVIRONMENTS_DIR="${HOME}/Jugglebot/environments"
else
  echo -e "\n[WARNING]: Specifying an alternate repo location is not supported. The '--debug-environments-dir' flag should only be used when testing this script.\n"
fi

JUGGLEBOT_CONDA_ENV_FILEPATH="${ENVIRONMENTS_DIR}/ubuntu-common/jugglebot_conda_env.yml"
SSH_PRIVATE_KEY_FILEPATH="${HOME}/.ssh/${SSH_KEYPAIR_NAME}"

task 'Enable ssh-agent'

eval "$(ssh-agent -s)"

task 'Add the ssh private key'

# Note: This will prompt for the passphrase if the key requires one 

ssh-add "${SSH_PRIVATE_KEY_FILEPATH}"

task 'Build the ansible-playbook command'

read -r -d '' ANSIBLE_PLAYBOOK_COMMAND << EndOfText || true
echo -e "\\nEnter your password to enable the ansible playbook to perform privileged operations" \\
&& ANSIBLE_LOCALHOST_WARNING=False ANSIBLE_INVENTORY_UNPARSED_WARNING=False ansible-playbook \\
"${ENVIRONMENTS_DIR}/ubuntu_24.04-wsl2/main_playbook.yml" \\
--ask-become-pass \\
-e upgrade_software=yes \\
-e "ssh_keypair_name='${SSH_KEYPAIR_NAME}'" \\
-e "git_name='${GIT_NAME}'" \\
-e "git_email='${GIT_EMAIL}'"
EndOfText

task 'Invoke ubuntu-common/base_setup.sh'

"${ENVIRONMENTS_DIR}/ubuntu-common/base_setup.sh" \
  --jugglebot-conda-env-filepath "${JUGGLEBOT_CONDA_ENV_FILEPATH}" \
  --ansible-playbook-command "${ANSIBLE_PLAYBOOK_COMMAND}"

