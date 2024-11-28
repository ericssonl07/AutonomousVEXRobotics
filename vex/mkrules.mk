# VEXcode mkrules.mk 2019_03_26_01

# compile C files
$(BUILD)/%.o: %.c $(SRC_H)
	$(Q)$(MKDIR)
	$(ECHO) "CC  $<"
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	
# compile C++ files
$(BUILD)/%.o: %.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q) g++ $(CXX_FLAGS) $(INC) -c -o $@ $<
#   $(Q) $(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<
	
# create executable
$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LINK) $(LNK_FLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

# create binary 
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# create archive
$(BUILD)/$(PROJECTLIB).a: $(OBJ)
	$(Q)$(ARCH) $(ARCH_FLAGS) $@ $^

# clean project
clean:
	$(info clean project)
	$(Q)$(CLEAN)


# # Project settings
# PROJECT = VEXRobotics
# BUILD = build
# SRC = src/autonomous/motor.cpp \
#       src/autonomous/odometry.cpp \
#       src/autonomous/pid.cpp \
#       src/autonomous/pursuit.cpp \
#       src/path/matrix.cpp \
#       src/path/path.cpp \
#       src/chassis.cpp \
#       src/main.cpp

# # Object files
# OBJ = $(SRC:src/%.cpp=$(BUILD)/%.o)

# # Compiler and flags
# CXX = g++
# CXX_FLAGS = -Os -Wall -Werror=return-type -fexceptions -std=c++20
# INC = -I"/Users/ericssonlin/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00/vexv5/clang/8.0.0/include" \
#       -I"/Users/ericssonlin/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00/vexv5/gcc/include/c++/4.9.3" \
#       -I"/Users/ericssonlin/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00/vexv5/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb" \
#       -I"/Users/ericssonlin/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00/vexv5/gcc/include/" \
#       -I"/Library/Developer/CommandLineTools/SDKs/MacOSX14.2.sdk/usr/include/c++/v1" \
#       -I.  # Add the current directory for local includes

# # Library paths
# LIBS = -L"/Users/ericssonlin/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00/vexv5/gcc/libs" -lv5rt

# # Targets

# # Compile C++ files
# $(BUILD)/%.o: src/%.cpp
# 	@mkdir -p $(dir $@)  # Ensure the build directory exists
# 	@echo "CXX $<"
# 	@$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

# # Create executable
# $(BUILD)/$(PROJECT).elf: $(OBJ)
# 	@echo "LINK $@"
# 	@$(CXX) $(CXX_FLAGS) -o $@ $^ $(LIBS)

# # Create binary
# $(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
# 	@$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# # Clean project
# clean:
# 	@echo "Cleaning project..."
# 	@rm -rf $(BUILD)/*.o $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# # Phony targets
# .PHONY: clean