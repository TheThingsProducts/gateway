#!/bin/bash

# make sure the generated change is not seen as something to commit
git update-index --assume-unchanged src/system_config/TTN_Gateway_v1/version.h

# get git commit
commit=$(git rev-parse HEAD)

export VERSION_TYPE=${VERSION_TYPE:-$(cat version/type)}
export VERSION_MAJOR=${VERSION_MAJOR:-$(cat version/major)}
export VERSION_MINOR=${VERSION_MINOR:-$(cat version/minor)}
export VERSION_PATCH=${VERSION_PATCH:-$(cat version/patch)}
export VERSION_COMMIT=${VERSION_COMMIT:-${commit:0:8}}
export VERSION_NAME=${VERSION_NAME:-$(cat version/name)}
export VERSION_TIMESTAMP=${VERSION_TIMESTAMP:-$(date +%s)}

# update the version.h with the values from version folder
echo "
#define VERSION_TYPE      ${VERSION_TYPE}
#define VERSION_MAJOR     ${VERSION_MAJOR}
#define VERSION_MINOR     ${VERSION_MINOR}
#define VERSION_PATCH     ${VERSION_PATCH}
#define VERSION_COMMIT    0x${VERSION_COMMIT}
#define VERSION_NAME      \"${VERSION_NAME}\"
#define VERSION_TIMESTAMP ${VERSION_TIMESTAMP}
" > src/system_config/TTN_Gateway_v1/version.h

echo "Generated version header:"
echo "*************************"
echo "Firmware name: ${VERSION_NAME}, type: ${VERSION_TYPE}, version: ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}, commit: ${VERSION_COMMIT}, timestamp: ${VERSION_TIMESTAMP} ($(date -d @${VERSION_TIMESTAMP} -u +%FT%TZ))"
