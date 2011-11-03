all: maps

FILENAME = willow.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/data/flirtlib/$(FILENAME)
SOURCE_DIR = build/prl
MD5SUM_FILE = willow.md5

include $(shell rospack find mk)/download_unpack_build.mk

maps: $(SOURCE_DIR)/unpacked
	rm -rf prl
	mv ${SOURCE_DIR} prl

clean: 
	rm -rf build
