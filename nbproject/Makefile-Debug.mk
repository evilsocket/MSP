#
# Gererated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/Debug/GNU-Linux-x86

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/examples/ai.o \
	${OBJECTDIR}/examples/motiondetection.o \
	${OBJECTDIR}/examples/fingerprint.o \
	${OBJECTDIR}/examples/gui.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS} dist/Debug/GNU-Linux-x86/msp

dist/Debug/GNU-Linux-x86/msp: ${OBJECTFILES}
	${MKDIR} -p dist/Debug/GNU-Linux-x86
	${LINK.cc} -o dist/Debug/GNU-Linux-x86/msp ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/examples/ai.o: examples/ai.cpp 
	${MKDIR} -p ${OBJECTDIR}/examples
	$(COMPILE.cc) -g -o ${OBJECTDIR}/examples/ai.o examples/ai.cpp

${OBJECTDIR}/examples/motiondetection.o: examples/motiondetection.cpp 
	${MKDIR} -p ${OBJECTDIR}/examples
	$(COMPILE.cc) -g -o ${OBJECTDIR}/examples/motiondetection.o examples/motiondetection.cpp

${OBJECTDIR}/examples/fingerprint.o: examples/fingerprint.cpp 
	${MKDIR} -p ${OBJECTDIR}/examples
	$(COMPILE.cc) -g -o ${OBJECTDIR}/examples/fingerprint.o examples/fingerprint.cpp

${OBJECTDIR}/examples/gui.o: examples/gui.cpp 
	${MKDIR} -p ${OBJECTDIR}/examples
	$(COMPILE.cc) -g -o ${OBJECTDIR}/examples/gui.o examples/gui.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/Debug
	${RM} dist/Debug/GNU-Linux-x86/msp

# Subprojects
.clean-subprojects:
