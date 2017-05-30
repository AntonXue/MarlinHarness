# C/C++ Compiler Information
CPP   = g++
FLAGS = -std=gnu++11 -fpack-struct -fshort-enums -ffunction-sections \
        -flto -g -Wno-ununsed-variable

# Build Information
BUILD_DIR = build
SRC_DIR   = src
OUTPUT    = harness
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
TGTS = $(subst $(SRC_DIR),$(BUILD_DIR),$(subst .cpp,.o,$(SRCS)))

all: clean $(BUILD_DIR) $(TGTS)
	$(CPP) $(FLAGS) $(TGTS) -o $(OUTPUT)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CPP) $(FLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(OUTPUT)

