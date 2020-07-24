################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_20.2.0.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT" --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT/Debug" --include_path="C:/ti/simplelink_cc32xx_sdk_4_10_00_07/source" --include_path="C:/ti/simplelink_cc32xx_sdk_4_10_00_07/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.3.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.3.1/FreeRTOS/Source/portable/CCS/ARM_CM3" --include_path="C:/Users/jrsav/workspace_v10/freertos_builds_CC3220SF_LAUNCHXL_release_ccs" --include_path="C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_20.2.0.LTS/include" -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT/Debug/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1416996079: ../common.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1000/ccs/utils/sysconfig_1.4.0/sysconfig_cli.bat" -s "C:/ti/simplelink_cc32xx_sdk_4_10_00_07/.metadata/product.json" -o "syscfg" --compiler ccs "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_net_config.c: build-1416996079 ../common.syscfg
syscfg/ti_drivers_net_wifi_config.c: build-1416996079
syscfg/ti_drivers_config.c: build-1416996079
syscfg/ti_drivers_config.h: build-1416996079
syscfg/syscfg_c.rov.xs: build-1416996079
syscfg/: build-1416996079

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_20.2.0.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT" --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT/Debug" --include_path="C:/ti/simplelink_cc32xx_sdk_4_10_00_07/source" --include_path="C:/ti/simplelink_cc32xx_sdk_4_10_00_07/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.3.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.3.1/FreeRTOS/Source/portable/CCS/ARM_CM3" --include_path="C:/Users/jrsav/workspace_v10/freertos_builds_CC3220SF_LAUNCHXL_release_ccs" --include_path="C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_20.2.0.LTS/include" -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/jrsav/workspace_v10/Full_Component_MQTT/Debug/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


