#!/usr/bin/env bash
set -o nounset -o pipefail -o errexit
IFS=$'\t\n' # Stricter IFS settings
rc=0

if [[ ! -d "${HOME}" ]]; then
  mv "/entrypoint/${USER}" "${HOME}"
fi

