#!/bin/bash

set -e -o pipefail

curl -L https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
echo "deb [arch=amd64] https://packages.microsoft.com/repos/azure-cli/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/azure-cli.list
apt-get update
apt-get install -y azure-cli jq
