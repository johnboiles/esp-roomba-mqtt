# Makefile
#
# Makefile for the Arduino Roomba project
#
# Author: Mike McCauley (mikem@airspayce.com)
# Copyright (C) 2010 Mike McCauley
# $Id: Makefile,v 1.1 2010/09/27 21:58:32 mikem Exp mikem $

PROJNAME = Roomba
# Dont forget to also change the version at the top of Roomba.h:
DISTFILE = $(PROJNAME)-1.3.zip

all:	doxygen dist upload

doxygen: 
	doxygen project.cfg


ci:
	ci -l `cat MANIFEST`

dist:	
	(cd ..; zip $(PROJNAME)/$(DISTFILE) `cat $(PROJNAME)/MANIFEST`)

upload:
	rsync -avz $(DISTFILE) doc/ www.airspayce.com:public_html/mikem/arduino/$(PROJNAME)
