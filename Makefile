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

# use open mp for mex files by default
OPENMP=1
ifeq ($(OPENMP),1)
    OPENMP_FLAG = -fopenmp
else
    OPENMP_FLAG =
endif  

MEX = mex
#MEX_FLAGS = -DDEFINEUNIX -largeArrayDims -lopenblas # maybe compile mtimesx using -lmwblas with is shipped by matlab
MEX_FLAGS = -DDEFINEUNIX -largeArrayDims -lmwblas
MEX_COPTIMFLAGS = CXXOPTIMFLAGS="-O3 $(OPENMP_FLAG)"
MEX_LDFLAGS = LDFLAGS="$(LDFLAGS) $(OPENMP_FLAG)"
MEX_LDOPTIMFLAGS = LDOPTIMFLAGS="$(LDOPTIMFLAGS) $(OPENMP_FLAG)"
MEX_CXXFLAGS = CXXFLAGS="$(CXXFLAGS) -std=c++14 -fPIC $(OPENMP_FLAG) -O3 -Wall -ffast-math"
MEX_BUILD_DIR = ../matlab/out
MEX_SRC_DIR = ../matlab/external/mtimesx
MEX_SRC = mtimesx

#armadillo requires blas/openblas and lapack (and not mwlapack)
MEX_ADD_SRC_DIR = $(MLIB_DIR)/Classes/Controllers/mex
ADD_STRUCTURE := $(shell find $(MEX_ADD_SRC_DIR) -type d)
MEX_ADD_SRC := $(addsuffix /*, $(ADD_STRUCTURE))
MEX_ADD_SRC := $(wildcard $(MEX_ADD_SRC))
MEX_ADD_SRC := $(filter %.cpp, $(MEX_ADD_SRC))
MEX_ADD_OUT_FILES := $(addprefix $(MEX_BUILD_DIR)/, $(notdir $(MEX_ADD_SRC:.cpp=.mexa64)))
#MEX_ADD_FLAGS = -DARMA_BLAS_LONG_LONG -DARMA_DONT_USE_WRAPPER -DARMA_NO_DEBUG -llapack -I$(MLIB_DIR)/external/armadillo/include -I$(MLIB_DIR)/external/armadillo/mex_interface
MEX_ADD_FLAGS = -DARMA_BLAS_LONG_LONG -DARMA_DONT_USE_WRAPPER -DARMA_NO_DEBUG -lmwlapack -I$(MLIB_DIR)/external/armadillo/include -I$(MLIB_DIR)/external/armadillo/mex_interface

VPATH=$(dir $(MEX_ADD_SRC))

print-%: ; @echo $*=$($*)

all: mex $(LIBNAME)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(LIBNAME): $(BUILD_DIR) $(BUILD_DIR)/$(LIBNAME).so

$(BUILD_DIR)/$(LIBNAME).so: $(SRC_FILES)
	$(MCC) $(MCC_FLAGS) -B cpplib:$(LIBNAME) $(API_FILES) -I $(SRC_DIR)/classes -I $(SRC_DIR)/functions -I $(MLIB_DIR)/Classes -I $(MLIB_DIR)/external -a $(MLIB_DIR)/Classes -a $(MEX_BUILD_DIR) -a $(MLIB_DIR)/external/yalmip -a $(MLIB_DIR)/external/NonlinearEstimationToolbox -a $(MLIB_DIR)/external/functions -d $(BUILD_DIR) 
	@rm -f $(BUILD_DIR)/$(LIBNAME).cpp

$(MEX_BUILD_DIR):
	@mkdir -p $(MEX_BUILD_DIR)

mex: $(MEX_BUILD_DIR) $(MEX_BUILD_DIR)/$(MEX_SRC).mexa64 $(MEX_ADD_OUT_FILES) 
    
$(MEX_BUILD_DIR)/$(MEX_SRC).mexa64: $(MEX_SRC_DIR)/$(MEX_SRC).c
	$(MEX) $(MEX_FLAGS) -output $@ $<

$(MEX_BUILD_DIR)/%.mexa64: %.cpp
	$(MEX) $(MEX_FLAGS) $(MEX_CXXFLAGS) $(MEX_ADD_FLAGS) $(MEX_COPTIMFLAGS) $(MEX_LDFLAGS) $(MEX_LDOPTIMFLAGS) -output $@ $<

clean cleanall:
	@rm -f $(BUILD_DIR)/$(LIBNAME).*
	@rm -f $(BUILD_DIR)/mccExcludedFiles.log
	@rm -f $(BUILD_DIR)/readme.txt
	@rm -f $(BUILD_DIR)/requiredMCRProducts.txt
	@rm -f $(MEX_BUILD_DIR)/*.mexa64
	
.PHONY: all lib clean mex

