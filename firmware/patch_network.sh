#!/bin/bash

replacements=(
    "s/KEEP_ALIVE_INTERVAL 20/KEEP_ALIVE_INTERVAL 60/"
    "s/COMMAND_TIMEOUT 2000/COMMAND_TIMEOUT 5000/"
    "s/QOS_STATUS QOS1/QOS_STATUS QOS0/"
    "s/QOS_DOWN QOS1/QOS_DOWN QOS0/"
    "s/QOS_UP QOS1/QOS_UP QOS0/"
    "s/QOS_CONNECT QOS1/QOS_CONNECT QOS0/"
    "s/QOS_WILL QOS1/QOS_WILL QOS0/"
)

for replacement in "${replacements[@]}";
do
    sed -i "${replacement}" ./vendor/ttn-gateway-connector/src/network.h
done
