#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

COMPONENT_ADD_INCLUDEDIRS := .
#COMPONENT_PRIV_INCLUDEDIRS :=

#EXTRA_CFLAGS := -DICACHE_RODATA_ATTR
#CFLAGS += -Wno-error=implicit-function-declaration -Wno-error=format= -DHAVE_CONFIG_H
COMPONENT_SRCDIRS := .