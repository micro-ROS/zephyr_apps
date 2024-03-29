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

CFLAGS_INTERNAL := $(CFLAGS)
CXXFLAGS_INTERNAL := $(CXXFLAGS)

ifeq ($(BOARD),native_posix)
	TOOLCHAIN = $(EXTENSIONS_DIR)/host_toolchain.cmake
	OBJECT_EXTENSION = o
else
	TOOLCHAIN = $(EXTENSIONS_DIR)/arm_toolchain.cmake
	OBJECT_EXTENSION = obj
	CFLAGS_INTERNAL := -c -I$(FW_DIR)/zephyrproject/zephyr/include/posix -I$(FW_DIR)/build/zephyr/include/generated $(CFLAGS_INTERNAL)
	CXXFLAGS_INTERNAL := -c -I$(FW_DIR)/zephyrproject/zephyr/include/posix -I$(FW_DIR)/build/zephyr/include/generated $(CXXFLAGS_INTERNAL)
endif

all: libmicroros

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

host_toolchain: $(EXTENSIONS_DIR)/host_toolchain.cmake.in
	rm -f $(EXTENSIONS_DIR)/host_toolchain.cmake; \
	cat $(EXTENSIONS_DIR)/host_toolchain.cmake.in | \
		sed "s/@CMAKE_C_COMPILER@/$(subst /,\/,$(CC))/g" | \
		sed "s/@CMAKE_CXX_COMPILER@/$(subst /,\/,$(CXX))/g" | \
		sed "s/@CMAKE_SYSROOT@/$(subst /,\/,$(FW_DIR))/g" | \
		sed "s/@CFLAGS@/$(subst /,\/,$(CFLAGS_INTERNAL))/g" | \
		sed "s/@CXXFLAGS@/$(subst /,\/,$(CXXFLAGS_INTERNAL))/g" \
		> $(EXTENSIONS_DIR)/host_toolchain.cmake

colcon_compile: arm_toolchain host_toolchain
	cd $(UROS_DIR); \
	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--metas $(UROS_DIR)/colcon.meta $(APP_META) \
		--cmake-args \
		-Wno-dev \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN) \
		-DCMAKE_VERBOSE_MAKEFILE=ON; \

get_package_names:
	@cd $(UROS_DIR); \
	colcon list | awk '{print $$1}' | awk -v d=";" '{s=(NR==1?s:s d)$$0}END{print s}'


libmicroros: colcon_compile
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/lib/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; $(AR) x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	$(AR) rc libmicroros.a *.$(OBJECT_EXTENSION); cp libmicroros.a ../install/; ranlib ../install/libmicroros.a; \
	cd ..; rm -rf libmicroros;