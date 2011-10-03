all: flirtlib

FILENAME = FLIRTLib.tgz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/FLIRTLib
MD5SUM_FILE = flirtlib.md5

include $(shell rospack find mk)/download_unpack_build.mk


flirtlib: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)/src; echo $(PWD); cmake .; make

clean: 
	rm -rf build