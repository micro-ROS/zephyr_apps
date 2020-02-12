EXTENSIONS_DIR = $(shell pwd)
FW_DIR = $(EXTENSIONS_DIR)/../..
UROS_DIR = $(FW_DIR)/mcu_ws
BUILD_DIR = $(EXTENSIONS_DIR)/build

PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib

DEBUG ?= 1

ifeq ($(DEBUG), 1)
  	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

$(warning PREFIX is $(PREFIX))
$(warning CC is $(CC))
$(warning AR is $(AR))
$(warning CFLAGS is $(CFLAGS))
$(warning INCLUDES is $(INCLUDES))
$(warning EXTENSIONS_DIR is $(EXTENSIONS_DIR))

# COLCON_INCLUDES_STR := -I$(FW_DIR)/zephyrproject/zephyr/include/posix 
# COLCON_INCLUDES_STR += $(foreach x,$(INCLUDES),$(x)\n)

CFLAGS_INTERNAL := $(CFLAGS)
CXXFLAGS_INTERNAL := $(CXXFLAGS)

CFLAGS_INTERNAL += -I$(FW_DIR)/zephyrproject/zephyr/include/posix
CXXFLAGS_INTERNAL += -I$(FW_DIR)/zephyrproject/zephyr/include/posix

# CXXFLAGS_INTERNAL += -isystem$(FW_DIR)/zephyr-sdk-0.11.1/arm-zephyr-eabi/arm-zephyr-eabi/include/c++/9.2.0
# CXXFLAGS_INTERNAL += -isystem$(FW_DIR)/zephyr-sdk-0.11.1/arm-zephyr-eabi/arm-zephyr-eabi/include/c++/9.2.0/arm-zephyr-eabi/thumb/v7e-m+fp/hard
# CXXFLAGS_INTERNAL += -isystem$(FW_DIR)/zephyr-sdk-0.11.1/arm-zephyr-eabi/arm-zephyr-eabi/include

all: libmicroros

# all: colcon_compile
# 	mkdir -p $(OBJ_DIR) $(LIB_DIR)
# 	$(CC) -c $(CFLAGS) -MD -Iinclude src/mylib.c -o $(OBJ_DIR)/mylib.o
# 	$(AR) -rcs $(LIB_DIR)/libmylib.a $(OBJ_DIR)/mylib.o

clean:
	rm -rf $(OBJ_DIR) $(LIB_DIR)

arm_toolchain: $(EXTENSIONS_DIR)/arm_toolchain.cmake.in
	rm -f $(EXTENSIONS_DIR)/arm_toolchain.cmake; \
	cat $(EXTENSIONS_DIR)/arm_toolchain.cmake.in | \
		sed "s/@CMAKE_C_COMPILER@/$(subst /,\/,$(CC))/g" | \
		sed "s/@CMAKE_CXX_COMPILER@/$(subst /,\/,$(CXX))/g" | \
		sed "s/@CMAKE_SYSROOT@/$(subst /,\/,$(FW_DIR))/g" | \
		sed "s/@CFLAGS@/$(subst /,\/,$(CFLAGS_INTERNAL))/g" | \
		sed "s/@CXXFLAGS@/$(subst /,\/,$(CXXFLAGS_INTERNAL))/g" \
		> $(EXTENSIONS_DIR)/arm_toolchain.cmake

colcon_compile: arm_toolchain
	cd $(UROS_DIR); \
	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--cmake-args \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/arm_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=ON; \

libmicroros: colcon_compile
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; $(AR) x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	$(AR) rc libmicroros.a *.obj; cp libmicroros.a ../install/; ranlib ../install/libmicroros.a; \
	cd ..; rm -rf libmicroros;