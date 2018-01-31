#!/bin/bash

#
# fix project for prjMakefilesGenerator
#

# replace c,S by c only otherwise prjMakefilesGenerator will fail
sed -i".bak" "s/\(<c-extensions>\)[^<>]*\(<\/c-extensions>\)/\1c\2/" TTN_Gateway.X/nbproject/project.xml
#!/bin/bash

#
# fix configuration paths after a run of Harmony
#

# read Harmony path which was used for generation
HARMONY_PATH_ORG=$(sed -ne '/HarmonyPath/ s/.*path=\"\(.*\)\".*/\1/p' src/system_config/TTN_Gateway_v1/configuration.xml)
# escape / \ and . characters for used in sed
HARMONY_PATH_ORG=$(echo $HARMONY_PATH_ORG | sed -e 's/\\/\\\\/g; s/\//\\\//g; s/\./\\\./g')
HARMONY_PATH_NEW="\.\.\/vendor\/harmony"
# replace generated Harmony path on the local system by the relative path in the repository
sed -i".bak" "/HarmonyPath/ s/\(.*path=\"\)\(.*\)\(\".*\)/\1$HARMONY_PATH_NEW\3/" src/system_config/TTN_Gateway_v1/configuration.xml
sed -i".bak" "s/\.\.\/\.\.\/$HARMONY_PATH_ORG/$HARMONY_PATH_NEW/g" TTN_Gateway.X/nbproject/configurations.xml
sed -i".bak" "s/\.\.\/\.\.\/$HARMONY_PATH_ORG/$HARMONY_PATH_NEW/g" src/system_config/TTN_Gateway_v1/configuration.xml
