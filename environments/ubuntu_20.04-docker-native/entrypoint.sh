#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

# TASK [Initialize the home directory VOLUME if necessary]

cp --no-clobber /etc/skel/.bashrc "${HOME}/.bashrc"
cp --no-clobber /etc/skel/.profile "${HOME}/.profile"
cp --no-clobber /etc/skel/.bash_logout "${HOME}/.bash_logout"

# TASK [Replace ~/.oh-my-zsh/custom with a symlink to the ~/.oh-my-zsh-custom VOLUME]

if [[ -d "${HOME}/.oh-my-zsh" ]]; then
  if [[ -d "${HOME}/.oh-my-zsh/custom" ]]; then
    if [[ ! -L "${HOME}/.oh-my-zsh/custom" ]]; then
      if [[ "$(ls -A "${HOME}/.oh-my-zsh/custom")" ]]; then
        echo -e "\n[WARNING]: The ~/.oh-my-zsh/custom directory is not empty.\n"
      else
        rmdir "${HOME}/.oh-my-zsh/custom"
        ln -s -T "${HOME}/.oh-my-zsh-custom" "${HOME}/.oh-my-zsh/custom"
      fi
    fi
  fi
fi

# TASK [Enable git to work within the Jugglebot VOLUME]

export GIT_DISCOVERY_ACROSS_FILESYSTEM=1

# TASK [Proceed with the CMD]

exec "$@"

