NATIVE_NAME = lib_trilateration
PYTHON_NAME = lib_trilateration_py
LIB_NAME = lib_trilateration

CC := gcc

CFLAGS := -c -fPIC

CLinkFlags = -shared -Wl,-soname,$@

PYTHON_VERSION := 2.7
PYTHON_INC := /usr/include/python$(PYTHON_VERSION)
BOOST_INC := /usr/include/boost
BOOST_LIB_LOCATION := /usr/lib/x86_64-linux-gnu
BOOST_PY_FILE := boost_python
BoostPyCInc := -I$(BOOST_INC) -I$(PYTHON_INC)
BoostPyLinkFlags = -Wl,-rpath,$(BOOST_LIB_LOCATION) -L$(BOOST_LIB_LOCATION) -l$(BOOST_PY_FILE)

PHONY: all
all: $(LIB_NAME).so $(PYTHON_NAME).so

$(LIB_NAME).so: $(NATIVE_NAME).o
	gcc $^ $(CLinkFlags) -o $@ -lstdc++

%.o: %.cpp
	gcc $(CFLAGS) $^ -o $@ -lstdc++

$(PYTHON_NAME).so: $(NATIVE_NAME).o $(PYTHON_NAME).o
	gcc $^ $(CLinkFlags) $(BoostPyLinkFlags) -o $@

$(PYTHON_NAME).o: $(PYTHON_NAME).cpp
	gcc $(CFLAGS) $(BoostPyCInc) $^ -o $@

clean:
	rm $(PYTHON_NAME).so
	rm $(PYTHON_NAME).o
	rm $(LIB_NAME).so
	rm $(NATIVE_NAME).o
