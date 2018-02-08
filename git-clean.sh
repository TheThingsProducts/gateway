#!/bin/bash

set -e
set -o pipefail

git clean -df
git submodule sync
git submodule update --init

