################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
PaLFI\ Demo.obj: ../PaLFI\ Demo.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"c:/ti/ccsv6/tools/compiler/msp430_4.3.3/bin/cl430" -vmsp --include_path="c:/ti/ccsv6/ccs_base/msp430/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" -g --define=__MSP430G2452__ --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="PaLFI Demo.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SPI_LowLevel.obj: ../SPI_LowLevel.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"c:/ti/ccsv6/tools/compiler/msp430_4.3.3/bin/cl430" -vmsp --include_path="c:/ti/ccsv6/ccs_base/msp430/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" -g --define=__MSP430G2452__ --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="SPI_LowLevel.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

SPI_Stack.obj: ../SPI_Stack.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"c:/ti/ccsv6/tools/compiler/msp430_4.3.3/bin/cl430" -vmsp --include_path="c:/ti/ccsv6/ccs_base/msp430/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.3/include" -g --define=__MSP430G2452__ --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="SPI_Stack.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


