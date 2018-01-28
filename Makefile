MCC = mcc
MCC_FLAGS = -v -g

SRC_DIR = matlab
API_DIR = $(SRC_DIR)/api
MLIB_DIR = ../matlab
BUILD_DIR = out
LIBNAME = libncs_matlab
API_FILES = $(wildcard $(API_DIR)/*.m) $(wildcard $(API_DIR)/*/*.m)
SRC_FILES = $(shell find $(SRC_DIR)/ -name '*.m')

all: $(LIBNAME)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(LIBNAME): $(BUILD_DIR) $(BUILD_DIR)/$(LIBNAME).so

$(BUILD_DIR)/$(LIBNAME).so: $(SRC_FILES)
	$(MCC) $(MCC_FLAGS) -B cpplib:$(LIBNAME) $(API_FILES) -I $(SRC_DIR)/classes -I $(SRC_DIR)/functions -I $(MLIB_DIR) -a $(MLIB_DIR) -d $(BUILD_DIR) 
	@rm -f $(BUILD_DIR)/$(LIBNAME).cpp

clean:
	@rm -f $(BUILD_DIR)/$(LIBNAME).*
	@rm -f $(BUILD_DIR)/mccExcludedFiles.log
	@rm -f $(BUILD_DIR)/readme.txt
	@rm -f $(BUILD_DIR)/requiredMCRProducts.txt

.PHONY: all lib clean

