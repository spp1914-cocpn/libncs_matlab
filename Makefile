MCC = mcc
MCC_FLAGS = -v
#MCC_FLAGS = -v -g

SRC_DIR = matlab
API_DIR = $(SRC_DIR)/api
MLIB_DIR = ../matlab
BUILD_DIR = out
LIBNAME = libncs_matlab
API_FILES = $(wildcard $(API_DIR)/*.m) $(wildcard $(API_DIR)/*/*.m)
SRC_FILES = $(shell find $(SRC_DIR)/ -name '*.m')
# the following variable contains files to be added to the deployable archive using mcc -a
MCC_ADD_FILES = $(MLIB_DIR)/Classes $(MEX_BUILD_DIR) $(MLIB_DIR)/external/yalmip $(MEX_BUILD_DIR) $(MLIB_DIR)/external/NonlinearEstimationToolbox $(wildcard $(MLIB_DIR)/external/sdpt3/*.m) $(wildcard $(MLIB_DIR)/external/sdpt3/*/*.m)
# the following variable contains folders to be included for the compilation using mcc -I
MCC_INC_DIRS = $(SRC_DIR)/classes $(SRC_DIR)/functions $(MLIB_DIR)/Classes $(MLIB_DIR)/external
MCC_ADD_FLAGS = $(addprefix -I , $(MCC_INC_DIRS)) $(addprefix -a , $(MCC_ADD_FILES))

#
# persistent make flags support
#
MAKECONF = ../makeconf

ifneq (,$(wildcard $(MAKECONF)))
	include $(MAKECONF)
endif

# use open mp for mex and armadillo files by default
OPENMP ?= 1
ifeq ($(OPENMP),1)
    OPENMP_FLAG = -fopenmp
    ARMA_OPENMP_FLAG =
else
    OPENMP_FLAG =
    ARMA_OPENMP_FLAG = -DARMA_DONT_USE_OPENMP
endif

MEX = mex
MEX_FLAGS = -DDEFINEUNIX -lmwblas -R2017b
MEX_COPTIMFLAGS = CXXOPTIMFLAGS="-O3 $(OPENMP_FLAG)"
MEX_LDFLAGS = LDFLAGS="$(LDFLAGS) $(OPENMP_FLAG)"
MEX_LDOPTIMFLAGS = LDOPTIMFLAGS="$(LDOPTIMFLAGS) $(OPENMP_FLAG)"
MEX_CXXFLAGS = CXXFLAGS="$(CXXFLAGS) -std=c++14 -fPIC $(OPENMP_FLAG) -O3 -Wall -ffast-math"

# mex files (C) of externals 
MEX_BUILD_DIR = $(MLIB_DIR)/$(BUILD_DIR)
MEX_SRC_DIR = $(MLIB_DIR)/external
MEX_SRC_DIR_SDPT = $(MEX_SRC_DIR)/sdpt3/Solver/Mexfun
MEX_SRC_DIR_MTIMESX = $(MEX_SRC_DIR)/mtimesx
MEX_SRC := $(MEX_SRC_DIR_MTIMESX)/mtimesx.c $(wildcard $(MEX_SRC_DIR_SDPT)/*.c)
MEX_OUT_FILES := $(addprefix $(MEX_BUILD_DIR)/, $(notdir $(MEX_SRC:.c=.mexa64)))

# own mex files (C++)
MEX_ADD_SRC_DIR = $(MLIB_DIR)/Classes/
MEX_ADD_SRC :=  $(shell find $(MEX_ADD_SRC_DIR) -wholename '*/mex/*/*.cpp')
MEX_ADD_OUT_FILES := $(addprefix $(MEX_BUILD_DIR)/, $(notdir $(MEX_ADD_SRC:.cpp=.mexa64)))
MEX_ADD_INC_DIRS = $(MLIB_DIR)/external/armadillo/include $(MLIB_DIR)/external/armadillo/mex_interface
MEX_ADD_FLAGS = $(ARMA_OPENMP_FLAG) -DARMA_BLAS_LONG_LONG -DARMA_DONT_USE_WRAPPER -DARMA_NO_DEBUG -lmwlapack -lmwblas $(addprefix -I, $(MEX_ADD_INC_DIRS))

VPATH=$(dir $(MEX_SRC)) $(dir $(MEX_ADD_SRC))

# prints the value of the mentioned variable
print-%: ; @echo $*=$($*)

all: mex $(LIBNAME)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(LIBNAME): $(BUILD_DIR) $(BUILD_DIR)/$(LIBNAME).so

$(BUILD_DIR)/$(LIBNAME).so: $(SRC_FILES)
	$(MCC) $(MCC_FLAGS) -B cpplib:$(LIBNAME) $(API_FILES) $(MCC_ADD_FLAGS) -d $(BUILD_DIR) 
	@rm -f $(BUILD_DIR)/$(LIBNAME).cpp

$(MEX_BUILD_DIR):
	@mkdir -p $(MEX_BUILD_DIR)/mex

mex:  $(MEX_BUILD_DIR) $(MEX_OUT_FILES) $(MEX_ADD_OUT_FILES) 

$(MEX_BUILD_DIR)/%.mexa64: %.c
	$(MEX) $(MEX_FLAGS) -output $@ $<

$(MEX_BUILD_DIR)/%.mexa64: %.cpp
	$(MEX) $(MEX_FLAGS) $(MEX_CXXFLAGS) $(MEX_ADD_FLAGS) $(MEX_COPTIMFLAGS) $(MEX_LDFLAGS) $(MEX_LDOPTIMFLAGS) -output $@ $<

cleanmex:
	@rm -f $(MEX_BUILD_DIR)/*.mexa64

clean cleanall: cleanmex
	@rm -f $(BUILD_DIR)/$(LIBNAME).*
	@rm -f $(BUILD_DIR)/mccExcludedFiles.log
	@rm -f $(BUILD_DIR)/readme.txt
	@rm -f $(BUILD_DIR)/requiredMCRProducts.txt
	@rm -f $(BUILD_DIR)/v2/generic_interface/$(LIBNAME).*
	@rm -f $(BUILD_DIR)/v2/generic_interface/readme.txt
	
.PHONY: all lib clean mex cleanmex

