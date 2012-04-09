all: flirtlib

SVN_REVISION = -r 6
SVN_URL = https://svn.openslam.org/data/svn/flirtlib
SVN_DIR = build/FLIRTLib
SVN_PATCH = flirtlib.patch

include $(shell rospack find mk)/svn_checkout.mk

flirtlib: download
	cd $(SVN_DIR)/trunk/src; echo $(PWD); cmake .; make

clean: 
	rm -rf build
