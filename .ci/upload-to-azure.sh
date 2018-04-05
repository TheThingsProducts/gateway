#!/bin/bash

set -e -o pipefail

if [[ -z "$AZURE_STORAGE_KEY" ]]
then
  exit 0
fi

container_name="the-things-gateway"
files="firmware.hex checksums firmware-with-bootloader.hex disassembly.txt"

upload() {
  echo "Uploading files to ${1}..."
  for file in ${files}
  do
    if [[ -f "$file" ]]
    then
      az storage blob upload --container-name "${container_name}" --name "v1/${1}/${file}" --file ./${file}
    fi
  done
}

if [[ ! -z "$TRAVIS_COMMIT" ]] then upload "$TRAVIS_COMMIT"; fi

if [[ "$TRAVIS_EVENT_TYPE" != "pull_request" && ! -z "$TRAVIS_BRANCH" ]] then upload "$TRAVIS_BRANCH"; fi
