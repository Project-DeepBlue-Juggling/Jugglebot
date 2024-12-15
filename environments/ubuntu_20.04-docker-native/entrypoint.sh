#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

INIT_FLAG_FILEPATH="${HOME}/.user-dir-initialized"

if [[ ! -f "${INIT_FLAG_FILEPATH}" ]]; then
  rm -rf "${HOME}"
  mv "/entrypoint/${USER}" "${HOME}"
  if [[ -d /entrypoint/oh-my-zsh-custom ]]; then
    rm -rf "${HOME}/.oh-my-zsh/custom"
    ln -s -T /entrypoint/oh-my-zsh-custom "${HOME}/.oh-my-zsh/custom"
  fi
  touch "${INIT_FLAG_FILEPATH}"
fi

exec "$@"

