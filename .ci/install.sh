#!/bin/bash

set -e -o pipefail

apt-get update
apt-get install -y apt-transport-https build-essential curl git lsb-release sudo
