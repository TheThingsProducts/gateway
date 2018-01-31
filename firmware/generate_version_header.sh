#!/bin/bash

# make sure the generated change is not seen as something to commit
git update-index --assume-unchanged src/system_config/TTN_Gateway_v1/version.h

# get git commit
commit=$(git rev-parse HEAD) 

# update the version.h with the values from version folder
echo "
#define VERSION_TYPE   $(cat version/type)
#define VERSION_MAJOR  $(cat version/major)
#define VERSION_MINOR  $(cat version/minor)
#define VERSION_PATCH  $(cat version/patch)
#define VERSION_COMMIT 0x${commit:0:8}
#define VERSION_NAME   \"$(cat version/name)\"
#define VERSION_TIMESTAMP $(date +%s)
" > src/system_config/TTN_Gateway_v1/version.h
