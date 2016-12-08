################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
source/gpio_if.obj: ../source/gpio_if.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O2 --fp_mode=relaxed --include_path="C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/MyBlink/includes" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/common/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/driverlib" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/inc" --define=cc3200 --diag_wrap=off --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="source/gpio_if.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/main.obj: ../source/main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O2 --fp_mode=relaxed --include_path="C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/MyBlink/includes" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/common/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/driverlib" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/inc" --define=cc3200 --diag_wrap=off --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="source/main.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/pinmux.obj: ../source/pinmux.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O2 --fp_mode=relaxed --include_path="C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/MyBlink/includes" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/common/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/driverlib" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/inc" --define=cc3200 --diag_wrap=off --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="source/pinmux.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/startup_ccs.obj: ../source/startup_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O2 --fp_mode=relaxed --include_path="C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/MyBlink/includes" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/common/include" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/driverlib" --include_path="H:/Data2/Gluevity/Technical/TIWorkspace/TIMiddlewareFiles/inc" --define=cc3200 --diag_wrap=off --diag_warning=225 --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="source/startup_ccs.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


