config SYS_PORTS_PPS_OUTPUT_${INSTANCE}
    depends on USE_SYS_PORTS 
    bool
    default y

ifblock SYS_PORTS_PPS_OUTPUT_${INSTANCE}


config USE_PPS_OUTPUT_${INSTANCE}
    bool

config SYS_PORT_PPS_OUTPUT_FUNCTION_${INSTANCE}
    string 
    range PORTS_REMAP_OUTPUT_FUNCTION

config SYS_PORT_PPS_OUTPUT_PIN_${INSTANCE}
    string 
    range PORTS_REMAP_OUTPUT_PIN

endif


	
