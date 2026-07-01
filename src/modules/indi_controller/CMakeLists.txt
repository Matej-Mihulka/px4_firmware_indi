px4_add_module(
    MODULE modules__indi_controller
    MAIN indi_controller
    STACK_MAIN 2000
    SRCS
        indi_controller.cpp
	MODULE_CONFIG
        module.yaml
)
