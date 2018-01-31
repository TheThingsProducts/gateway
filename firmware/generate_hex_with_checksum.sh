#!/bin/bash

set -e

cp TTN_Gateway.X/dist/TTN_Gateway_v1/production/TTN_Gateway.X.production.hex firmware.hex
sha256sum firmware.hex > checksums

