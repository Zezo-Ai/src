# $OpenBSD: Makefile,v 1.8 2025/06/11 17:46:32 robert Exp $

LIB=	LLVMAMDGPUCodeGen
NOPROFILE=

CPPFLAGS+=	-I${.OBJDIR}/../include/llvm/AMDGPU \
		-I${LLVM_SRCS}/lib/Target/AMDGPU

SRCS+=	AMDGPUAliasAnalysis.cpp \
	AMDGPUAlwaysInlinePass.cpp \
	AMDGPUAnnotateKernelFeatures.cpp \
	AMDGPUAnnotateUniformValues.cpp \
	AMDGPUArgumentUsageInfo.cpp \
	AMDGPUAsmPrinter.cpp \
	AMDGPUAtomicOptimizer.cpp \
	AMDGPUAttributor.cpp \
	AMDGPUCallLowering.cpp \
	AMDGPUCodeGenPassBuilder.cpp \
	AMDGPUCodeGenPrepare.cpp \
	AMDGPUCombinerHelper.cpp \
	AMDGPUCtorDtorLowering.cpp \
	AMDGPUExportClustering.cpp \
	AMDGPUFrameLowering.cpp \
	AMDGPUGlobalISelDivergenceLowering.cpp \
	AMDGPUGlobalISelUtils.cpp \
	AMDGPUHSAMetadataStreamer.cpp \
	AMDGPUInsertDelayAlu.cpp \
	AMDGPUInstCombineIntrinsic.cpp \
	AMDGPUInstrInfo.cpp \
	AMDGPUInstructionSelector.cpp \
	AMDGPUISelDAGToDAG.cpp \
	AMDGPUISelLowering.cpp \
	AMDGPULateCodeGenPrepare.cpp \
	AMDGPULegalizerInfo.cpp \
	AMDGPULibCalls.cpp \
	AMDGPUImageIntrinsicOptimizer.cpp \
	AMDGPULibFunc.cpp \
	AMDGPULowerBufferFatPointers.cpp \
	AMDGPULowerKernelArguments.cpp \
	AMDGPULowerKernelAttributes.cpp \
	AMDGPULowerModuleLDSPass.cpp \
	AMDGPUMachineCFGStructurizer.cpp \
	AMDGPUMachineFunction.cpp \
	AMDGPUMachineModuleInfo.cpp \
	AMDGPUMacroFusion.cpp \
	AMDGPUMCInstLower.cpp \
	AMDGPUIGroupLP.cpp \
	AMDGPUInsertSingleUseVDST.cpp \
	AMDGPUMarkLastScratchLoad.cpp \
	AMDGPUMIRFormatter.cpp \
	AMDGPUOpenCLEnqueuedBlockLowering.cpp \
	AMDGPUPerfHintAnalysis.cpp \
	AMDGPUPostLegalizerCombiner.cpp \
	AMDGPUPreLegalizerCombiner.cpp \
	AMDGPUPrintfRuntimeBinding.cpp \
	AMDGPUPromoteAlloca.cpp \
	AMDGPUPromoteKernelArguments.cpp \
	AMDGPURegBankCombiner.cpp \
	AMDGPURegBankSelect.cpp \
	AMDGPURegisterBankInfo.cpp \
	AMDGPURemoveIncompatibleFunctions.cpp \
	AMDGPUResourceUsageAnalysis.cpp \
	AMDGPURewriteOutArguments.cpp \
	AMDGPURewriteUndefForPHI.cpp \
	AMDGPUSetWavePriority.cpp \
	AMDGPUSplitModule.cpp \
	AMDGPUSubtarget.cpp \
	AMDGPUTargetMachine.cpp \
	AMDGPUTargetObjectFile.cpp \
	AMDGPUTargetTransformInfo.cpp \
	AMDGPUUnifyDivergentExitNodes.cpp \
	AMDGPUUnifyMetadata.cpp \
	R600MachineCFGStructurizer.cpp \
	GCNCreateVOPD.cpp \
	GCNDPPCombine.cpp \
	GCNHazardRecognizer.cpp \
	GCNILPSched.cpp \
	GCNIterativeScheduler.cpp \
	GCNMinRegStrategy.cpp \
	GCNNSAReassign.cpp \
	GCNPreRAOptimizations.cpp \
	GCNPreRALongBranchReg.cpp \
	GCNRegPressure.cpp \
	GCNRewritePartialRegUses.cpp \
	GCNSchedStrategy.cpp \
	GCNVOPDUtils.cpp \
	R600AsmPrinter.cpp \
	R600ClauseMergePass.cpp \
	R600CodeGenPassBuilder.cpp \
	R600ControlFlowFinalizer.cpp \
	R600EmitClauseMarkers.cpp \
	R600ExpandSpecialInstrs.cpp \
	R600FrameLowering.cpp \
	R600InstrInfo.cpp \
	R600ISelDAGToDAG.cpp \
	R600ISelLowering.cpp \
	R600MachineFunctionInfo.cpp \
	R600MachineScheduler.cpp \
	R600MCInstLower.cpp \
	R600OpenCLImageTypeLoweringPass.cpp \
	R600OptimizeVectorRegisters.cpp \
	R600Packetizer.cpp \
	R600RegisterInfo.cpp \
	R600Subtarget.cpp \
	R600TargetMachine.cpp \
	R600TargetTransformInfo.cpp \
	SIAnnotateControlFlow.cpp \
	SIFixSGPRCopies.cpp \
	SIFixVGPRCopies.cpp \
	SIFoldOperands.cpp \
	SIFormMemoryClauses.cpp \
	SIFrameLowering.cpp \
	SIInsertHardClauses.cpp \
	SIInsertWaitcnts.cpp \
	SIInstrInfo.cpp \
	SIISelLowering.cpp \
	SILateBranchLowering.cpp \
	SILoadStoreOptimizer.cpp \
	SILowerControlFlow.cpp \
	SILowerI1Copies.cpp \
	SILowerWWMCopies.cpp \
	SILowerSGPRSpills.cpp \
	SIMachineFunctionInfo.cpp \
	SIMachineScheduler.cpp \
	SIMemoryLegalizer.cpp \
	SIModeRegister.cpp \
	SIModeRegisterDefaults.cpp \
	SIOptimizeExecMasking.cpp \
	SIOptimizeExecMaskingPreRA.cpp \
	SIOptimizeVGPRLiveRange.cpp \
	SIPeepholeSDWA.cpp \
	SIPostRABundler.cpp \
	SIPreAllocateWWMRegs.cpp \
	SIPreEmitPeephole.cpp \
	SIProgramInfo.cpp \
	SIRegisterInfo.cpp \
	SIShrinkInstructions.cpp \
	SIWholeQuadMode.cpp

.PATH:	${.CURDIR}/../../../llvm/llvm/lib/Target/AMDGPU
