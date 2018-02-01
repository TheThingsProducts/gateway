Compile TTN Gateway Firmware on Ubuntu
=================================

Install 64 Bit dependencies
---------------------------
Resource: http://microchipdeveloper.com/install:mplabx-lin64

    sudo apt-get install libc6:i386 libx11-6:i386 libxext6:i386 \
    libstdc++6:i386 libexpat1:i386

Download and install MPLABX
---------------------------
Resource: http://microchipdeveloper.com/mplabx:installation

    tar -xvf MPLABX-v4.05-linux-installer.tar
    chmod u+x MPLABX-v4.05-linux-installer.sh
    sudo ./MPLABX-v4.05-linux-installer.sh

Next, Accept Agreement, Next
/opt/microchip/mplabx/v4.05
Next, Finish

To prevent error java not found:

    sudo ln -s /opt/microchip/mplabx/v4.05/sys /opt/microchip/mplab_ide/sys


Install XS32 compiler
---------------------

Download
Resource: http://www.microchip.com/mplabxc32linux

    NOTE: we need older version v1.42 to prevent errors like these:
    error: '_CNCONB_SIDL_MASK' undeclared (first use in this function..)
    You get them when you try to compile with 1.43 or higher.

https://www.webeaver.com/1199815/08f0d806ca3a4d6a7dc32c4344237875/download-xc32-v1-42-full-install-linux-installer-run
If anyone find a safer download source, please let me know.

    chmod +x ./xc32-v1.42-full-install-linux-installer.run 
    sudo ./xc32-v1.42-full-install-linux-installer.run 

Next, Accept, Next, Next
/opt/microchip/xc32/v1.42
Next, Add path, Next, Next, Get Coffee, Next, Finish

    sudo apt install cppcheck

    mkdir ~/ttn-gw-dev && cd ~/ttn-gw-dev

    git clone --recursive https://github.com/TheThingsProducts/gateway.git
    cd gateway/firmware

    cppcheck --enable=all --inconclusive --force --xml --xml-version=2 src 2> cppcheck.xml

    ./generate_version_header.sh
    ./fix_mplab.sh

    vi compile.sh and correct prjMakefilesGenerator path:

    elif [ "$(uname -s)" = "Linux" ]; then
      prjMakefilesGenerator='/opt/microchip/mplabx/v4.05/mplab_ide/bin/prjMakefilesGenerator.sh'

    ./compile.sh 

[Get Coffee here]

    ./generate_hex_with_checksum.sh

    ls -l firmware.hex 
    -rw-r--r-- 1 ron ron 2767956 feb  1 21:22 firmware.hex

    cat checksums 
    337950176cab81e5715b16c86af424cca57d0de088e5a24dcec7fef5c24e432b  firmware.hex

Write them to SD Card (I used a 2GB FAT32 formatted one)

    mkdir /media/<your path>/update

    cp firmware.hex checksums /media/<your path>/update

    ls -l /media/ron/<your path>/update/
    total 2708
    -rw-r--r-- 1 ron ron      79 feb  1 21:52 checksums
    -rw-r--r-- 1 ron ron 2767956 feb  1 21:52 firmware.hex

Insert in gateway and boot
Notice the intermittent outside and center leds blinking until finished.

Check you info page at http://things-gateway.local/info
