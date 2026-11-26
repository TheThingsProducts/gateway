#!/bin/bash
# Convert a PEM certificate to a C array for net_pres_cert_store.c
# Usage: ./update_certs.sh <cert.pem> <SYMBOL_NAME>
set -e
PEM=$1
SYMBOL=$2
openssl x509 -in "$PEM" -inform PEM -out cert.der -outform DER
srec_cat cert.der -binary -output "${SYMBOL}.c" -c-Array "$SYMBOL" -include
echo "Generated ${SYMBOL}.c — copy the array into net_pres_cert_store.c"
rm cert.der
