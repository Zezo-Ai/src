# $OpenBSD: Makefile,v 1.13 2025/06/11 17:46:33 robert Exp $

LIB=	lldbPluginProcess
NOPIC=
NOPROFILE=

.include <bsd.own.mk>

# Process/OpenBSD
SRCS=	NativeProcessOpenBSD.cpp \
	NativeRegisterContextOpenBSD.cpp \
	NativeRegisterContextOpenBSD_arch.cpp \
	NativeRegisterContextOpenBSD_arm64.cpp \
	NativeRegisterContextOpenBSD_x86_64.cpp \
	NativeThreadOpenBSD.cpp

# Process/POSIX
SRCS+=	CrashReason.cpp \
	NativeProcessELF.cpp \
	ProcessPOSIXLog.cpp

# Process/gdb-remote
SRCS+=	GDBRemoteClientBase.cpp \
	GDBRemoteCommunication.cpp \
	GDBRemoteCommunicationClient.cpp \
	GDBRemoteCommunicationHistory.cpp \
	GDBRemoteCommunicationServer.cpp \
	GDBRemoteCommunicationServerCommon.cpp \
	GDBRemoteCommunicationServerLLGS.cpp \
	GDBRemoteCommunicationServerPlatform.cpp \
	GDBRemoteRegisterContext.cpp \
	GDBRemoteRegisterFallback.cpp \
	ProcessGDBRemote.cpp \
	ProcessGDBRemoteLog.cpp \
	ThreadGDBRemote.cpp

# Process/Utility
SRCS+=	AuxVector.cpp \
	FreeBSDSignals.cpp \
	GDBRemoteSignals.cpp \
	HistoryThread.cpp \
	HistoryUnwind.cpp \
	InferiorCallPOSIX.cpp \
	LinuxProcMaps.cpp \
	LinuxSignals.cpp \
	MemoryTagManagerAArch64MTE.cpp \
	NativeProcessSoftwareSingleStep.cpp \
	NativeRegisterContextDBReg_arm64.cpp \
	NativeRegisterContextDBReg_x86.cpp \
	NativeRegisterContextRegisterInfo.cpp \
	NetBSDSignals.cpp \
	OpenBSDSignals.cpp \
	RegisterContext_x86.cpp \
	RegisterContextDarwin_arm.cpp \
	RegisterContextDarwin_arm64.cpp \
	RegisterContextDarwin_i386.cpp \
	RegisterContextDarwin_x86_64.cpp \
	RegisterContextDummy.cpp \
	RegisterContextFreeBSD_i386.cpp \
	RegisterContextFreeBSD_mips64.cpp \
	RegisterContextFreeBSD_powerpc.cpp \
	RegisterContextFreeBSD_x86_64.cpp \
	RegisterContextHistory.cpp \
	RegisterContextLinux_i386.cpp \
	RegisterContextLinux_x86_64.cpp \
	RegisterContextLinux_s390x.cpp \
	RegisterContextMach_arm.cpp \
	RegisterContextMach_i386.cpp \
	RegisterContextMach_x86_64.cpp \
	RegisterContextMemory.cpp \
	RegisterContextNetBSD_i386.cpp \
	RegisterContextNetBSD_x86_64.cpp \
	RegisterContextOpenBSD_i386.cpp \
	RegisterContextOpenBSD_x86_64.cpp \
	RegisterContextPOSIX_arm.cpp \
	RegisterContextPOSIX_arm64.cpp \
	RegisterContextPOSIX_loongarch64.cpp \
	RegisterContextPOSIX_mips64.cpp \
	RegisterContextPOSIX_powerpc.cpp \
	RegisterContextPOSIX_ppc64le.cpp \
	RegisterContextPOSIX_riscv64.cpp \
	RegisterContextPOSIX_s390x.cpp \
	RegisterContextPOSIX_x86.cpp \
	RegisterContextThreadMemory.cpp \
	RegisterContextWindows_i386.cpp \
	RegisterFlagsDetector_arm64.cpp \
	RegisterInfos_x86_64_with_base_shared.cpp \
	RegisterContextWindows_x86_64.cpp \
	RegisterInfoPOSIX_arm.cpp \
	RegisterInfoPOSIX_arm64.cpp \
	RegisterInfoPOSIX_loongarch64.cpp \
	RegisterInfoPOSIX_ppc64le.cpp \
	RegisterInfoPOSIX_riscv64.cpp \
	StopInfoMachException.cpp \
	ThreadMemory.cpp

# Process/elf-core
SRCS+=	ProcessElfCore.cpp \
	ThreadElfCore.cpp \
	RegisterContextLinuxCore_x86_64.cpp \
	RegisterContextPOSIXCore_arm.cpp \
	RegisterContextPOSIXCore_arm64.cpp \
	RegisterContextPOSIXCore_mips64.cpp \
	RegisterContextPOSIXCore_powerpc.cpp \
	RegisterContextPOSIXCore_ppc64le.cpp \
	RegisterContextPOSIXCore_s390x.cpp \
	RegisterContextPOSIXCore_x86_64.cpp \
	RegisterContextPOSIXCore_riscv64.cpp \
	RegisterUtilities.cpp

# Process/mach-core
SRCS+=	ProcessMachCore.cpp \
	ThreadMachCore.cpp

# Process/minidump
SRCS+=	MinidumpTypes.cpp \
	MinidumpParser.cpp \
	RegisterContextMinidump_ARM.cpp \
	RegisterContextMinidump_ARM64.cpp \
	RegisterContextMinidump_x86_32.cpp \
	RegisterContextMinidump_x86_64.cpp \
	ProcessMinidump.cpp \
	ThreadMinidump.cpp

# Process/scripted
SRCS+=	ScriptedProcess.cpp \
	ScriptedThread.cpp

# Process/OpenBSDKernel
SRCS+=	ProcessOpenBSDKernel.cpp \
	RegisterContextOpenBSDKernel_arm64.cpp \
	RegisterContextOpenBSDKernel_i386.cpp \
	RegisterContextOpenBSDKernel_x86_64.cpp \
	ThreadOpenBSDKernel.cpp

.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/OpenBSD
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/POSIX
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/gdb-remote
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/Utility
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/elf-core
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/mach-core
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/minidump
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/scripted
.PATH:	${.CURDIR}/../../../llvm/lldb/source/Plugins/Process/OpenBSDKernel

install:
	@# Nothing here so far ...

.include <bsd.lib.mk>

CPPFLAGS+=	${LLDB_INCLUDES}
CPPFLAGS+=	${CLANG_INCLUDES}
CPPFLAGS+=	-I${.OBJDIR}/../include/lldb/Plugins
