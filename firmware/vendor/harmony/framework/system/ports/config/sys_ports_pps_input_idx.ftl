config SYS_PORTS_PPS_INPUT_${INSTANCE}
    depends on USE_SYS_PORTS 
    bool
    default y

ifblock SYS_PORTS_PPS_INPUT_${INSTANCE}

config USE_PPS_INPUT_${INSTANCE}
    bool

config SYS_PORT_PPS_INPUT_FUNCTION_${INSTANCE}
    string 
    range PORTS_REMAP_INPUT_FUNCTION

config SYS_PORT_PPS_INPUT_PIN_${INSTANCE}
    string 
    range PORTS_REMAP_INPUT_PIN

endif


	
