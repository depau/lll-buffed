#!/usr/bin/env bash
set -euo pipefail

#exec 19>/tmp/bash-trace.log
#export BASH_XTRACEFD=19
#set -x

# Remove all entries from PATH mentioning platformio
PATH_ARR=()
IFS=':' read -ra PATH_ARR <<<"$PATH"
NEW_PATH_ARR=()

for entry in "${PATH_ARR[@]}"; do
  if [[ "$entry" != *"platformio"* && "$entry" != *"toolchains/system-avr-gcc/bin"* ]]; then
    NEW_PATH_ARR+=("$entry")
  fi
done

NEW_PATH=$(
  IFS=:
  echo "${NEW_PATH_ARR[*]}"
)

export PATH="$NEW_PATH"

# Execute the actual compiler with the provided arguments
exec "$(basename "$0")" "$@"
