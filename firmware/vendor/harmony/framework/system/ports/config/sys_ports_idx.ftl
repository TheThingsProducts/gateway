config SYS_PORTS_INST_IDX${INSTANCE}
    depends on USE_SYS_PORTS 
    bool
    default y

ifblock SYS_PORTS_INST_IDX${INSTANCE}

<#assign port = ["A", "B", "C", "D", "E", "F", "G", "H", "J", "K"]>

config USE_PORT_${port[INSTANCE]}
    bool

config SYS_PORT_${port[INSTANCE]}_ANSEL
    hex
    default 0xFFFF

config SYS_PORT_${port[INSTANCE]}_TRIS
    hex
    default 0xFFFF

config SYS_PORT_${port[INSTANCE]}_LAT
    hex
    default 0x0

config SYS_PORT_${port[INSTANCE]}_ODC
    hex
    default 0x0

config SYS_PORT_${port[INSTANCE]}_CNPU
    hex
    default 0x0

config SYS_PORT_${port[INSTANCE]}_CNPD
    hex
    default 0x0

config SYS_PORT_${port[INSTANCE]}_CNEN
    hex
    default 0x0

endif
