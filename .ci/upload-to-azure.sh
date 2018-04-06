#!/bin/bash

set -e -o pipefail

if [[ -z "$AZURE_STORAGE_KEY" ]]
then
  exit 0
fi

if [[ -z "$TRAVIS_COMMIT" ]]
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

copy() {
  echo "Copying files from ${1} to ${2}..."
  for file in ${files}
  do
    az storage blob copy start --source-container "${container_name}" --destination-container "${container_name}" --source-blob "v1/${1}/${file}" --destination-blob "v1/${2}/${file}"
  done
}

if [[ $(az storage blob exists --container-name "${container_name}" --name "v1/$TRAVIS_COMMIT/checksums" | jq '.exists') != 'true' ]]
then
  upload "$TRAVIS_COMMIT"
fi

if [[ "$TRAVIS_EVENT_TYPE" != "pull_request" && ! -z "$TRAVIS_BRANCH" ]]
then
  copy "$TRAVIS_COMMIT" "$TRAVIS_BRANCH"
fi
