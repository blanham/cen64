//
// vr4300/opcodes.h: VR4300 opcode types and info.
//
// CEN64: Cycle-Accurate Nintendo 64 Simulator.
// Copyright (C) 2014, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef __vr4300_opcodes_h__
#define __vr4300_opcodes_h__
#include "common.h"

enum vr4300_opcode_id {
#define X(op) VR4300_OPCODE_##op,
#include "vr4300/opcodes.md"
  NUM_VR4300_OPCODES
#undef X
};

struct vr4300;
typedef void (*const vr4300_function)(struct vr4300 *, uint64_t, uint64_t);

extern const vr4300_function vr4300_function_table[NUM_VR4300_OPCODES];
extern const char *vr4300_opcode_mnemonics[NUM_VR4300_OPCODES];

/* Flags for each instruction. */
#ifdef VR4300_BUILD_FUNCS
#define VR4300_BUILD_OP(op, func, flags) \
  (VR4300_##func)
#else
#define VR4300_BUILD_OP(op, func, flags) \
  (VR4300_OPCODE_##op), (flags)
#endif

#define INFO1(x) (OPCODE_INFO_##x)
#define INFO2(x,y) (INFO1(x) | OPCODE_INFO_##y)
#define INFO3(x,y,z) (INFO2(x,y) | OPCODE_INFO_##z)
#define INFO4(x,y,z,a) (INFO3(x,y,z) | OPCODE_INFO_##a)
#define INFO5(x,y,z,a,b) (INFO4(x,y,z,a) | OPCODE_INFO_##b)
#define INVALID VR4300_BUILD_OP(INVALID, INV, INFO1(NONE))

#define BC0 VR4300_BUILD_OP(BC0, INV, INFO1(BRANCH))
#define BC1 VR4300_BUILD_OP(BC1, INV, INFO1(BRANCH))
#define BC2 VR4300_BUILD_OP(BC1, INV, INFO1(BRANCH))
#define ERET VR4300_BUILD_OP(ERET, ERET, INFO1(NONE))
#define FPUS VR4300_BUILD_OP(FPUS, INV, INFO1(NONE))
#define FPUD VR4300_BUILD_OP(FPUD, INV, INFO1(NONE))
#define FPUW VR4300_BUILD_OP(FPUW, INV, INFO1(NONE))
#define FPUL VR4300_BUILD_OP(FPUL, INV, INFO1(NONE))
#define TLBP VR4300_BUILD_OP(TLBP, INV, INFO1(NONE))
#define TLBR VR4300_BUILD_OP(TLBR, INV, INFO1(NONE))
#define TLBWI VR4300_BUILD_OP(TLBWI, INV, INFO1(NONE))
#define TLBWR VR4300_BUILD_OP(TLBWR, INV, INFO1(NONE))

#define ADD VR4300_BUILD_OP(ADD, ADD_SUB, INFO2(NEEDRS, NEEDRT))
#define ADDI VR4300_BUILD_OP(ADDI, ADDI_SUBI, INFO1(NEEDRS))
#define ADDIU VR4300_BUILD_OP(ADDIU, ADDIU_SUBIU, INFO1(NEEDRS))
#define ADDU VR4300_BUILD_OP(ADDU, ADDU_SUBU, INFO2(NEEDRS, NEEDRT))
#define AND VR4300_BUILD_OP(AND, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define ANDI VR4300_BUILD_OP(ANDI, ANDI_ORI_XORI, INFO1(NEEDRS))
#define BEQ VR4300_BUILD_OP(BEQ, BEQ_BEQL_BNE_BNEL, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BEQL VR4300_BUILD_OP(BEQL, BEQ_BEQL_BNE_BNEL, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BGEZ VR4300_BUILD_OP(BGEZ, BGEZ_BGEZL_BLTZ_BLTZL, INFO2(BRANCH, NEEDRS))
#define BGEZAL VR4300_BUILD_OP(BGEZAL, BGEZAL_BGEZALL_BLTZAL_BLTZALL, INFO2(BRANCH, NEEDRS))
#define BGEZALL VR4300_BUILD_OP(BGEZALL, BGEZAL_BGEZALL_BLTZAL_BLTZALL, INFO2(BRANCH, NEEDRS))
#define BGEZL VR4300_BUILD_OP(BGEZL, BGEZ_BGEZL_BLTZ_BLTZL, INFO2(BRANCH, NEEDRS))
#define BGTZ VR4300_BUILD_OP(BGTZ, BGTZ_BGTZL_BLEZ_BLEZL, INFO2(BRANCH, NEEDRS))
#define BGTZL VR4300_BUILD_OP(BGTZL, BGTZ_BGTZL_BLEZ_BLEZL, INFO2(BRANCH, NEEDRS))
#define BLEZ VR4300_BUILD_OP(BLEZ, BGTZ_BGTZL_BLEZ_BLEZL, INFO2(BRANCH, NEEDRS))
#define BLEZL VR4300_BUILD_OP(BLEZL, BGTZ_BGTZL_BLEZ_BLEZL, INFO2(BRANCH, NEEDRS))
#define BLTZ VR4300_BUILD_OP(BLTZ, BGEZ_BGEZL_BLTZ_BLTZL, INFO2(BRANCH, NEEDRS))
#define BLTZAL VR4300_BUILD_OP(BLTZAL, BGEZAL_BGEZALL_BLTZAL_BLTZALL, INFO2(BRANCH, NEEDRS))
#define BLTZALL VR4300_BUILD_OP(BLTZALL, BGEZAL_BGEZALL_BLTZAL_BLTZALL, INFO2(BRANCH, NEEDRS))
#define BLTZL VR4300_BUILD_OP(BLTZL, BGEZ_BGEZL_BLTZ_BLTZL, INFO2(BRANCH, NEEDRS))
#define BNE VR4300_BUILD_OP(BNE, BEQ_BEQL_BNE_BNEL, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BNEL VR4300_BUILD_OP(BNEL, BEQ_BEQL_BNE_BNEL, INFO3(BRANCH, NEEDRS, NEEDRT))
#define BREAK VR4300_BUILD_OP(BREAK, INV, INFO1(NONE))
#define CACHE VR4300_BUILD_OP(CACHE, INV, INFO1(NONE))
#define CFC0 VR4300_BUILD_OP(CFC0, INV, INFO1(NONE))
#define CFC1 VR4300_BUILD_OP(CFC1, CFC1, INFO1(NONE))
#define CFC2 VR4300_BUILD_OP(CFC2, INV, INFO1(NONE))
#define COP0 VR4300_BUILD_OP(COP0, INV, INFO1(NONE))
#define COP1 VR4300_BUILD_OP(COP1, INV, INFO1(NONE))
#define COP2 VR4300_BUILD_OP(COP2, INV, INFO1(NONE))
#define CTC0 VR4300_BUILD_OP(CTC0, INV, INFO1(NONE))
#define CTC1 VR4300_BUILD_OP(CTC1, CTC1, INFO1(NEEDRT))
#define CTC2 VR4300_BUILD_OP(CTC2, INV, INFO1(NONE))
#define DADD VR4300_BUILD_OP(DADD, INV, INFO1(NONE))
#define DADDI VR4300_BUILD_OP(DADDI, INV, INFO1(NONE))
#define DADDIU VR4300_BUILD_OP(DADDIU, INV, INFO1(NONE))
#define DADDU VR4300_BUILD_OP(DADDU, INV, INFO1(NONE))
#define DDIV VR4300_BUILD_OP(DDIV, INV, INFO1(NONE))
#define DDIVU VR4300_BUILD_OP(DDIVU, DDIVU, INFO2(NEEDRS, NEEDRT))
#define DIV VR4300_BUILD_OP(DIV, DIV_DIVU, INFO2(NEEDRS, NEEDRT))
#define DIVU VR4300_BUILD_OP(DIVU, DIV_DIVU, INFO2(NEEDRS, NEEDRT))
#define DMFC0 VR4300_BUILD_OP(DMFC0, INV, INFO1(NONE))
#define DMFC1 VR4300_BUILD_OP(DMFC1, INV, INFO1(NONE))
#define DMFC2 VR4300_BUILD_OP(DMFC2, INV, INFO1(NONE))
#define DMTC0 VR4300_BUILD_OP(DMTC0, INV, INFO1(NONE))
#define DMTC1 VR4300_BUILD_OP(DMTC1, INV, INFO1(NONE))
#define DMTC2 VR4300_BUILD_OP(DMTC2, INV, INFO1(NONE))
#define DMULT VR4300_BUILD_OP(DMULT, INV, INFO1(NONE))
#define DMULTU VR4300_BUILD_OP(DMULTU, DMULTU, INFO2(NEEDRS, NEEDRT))
#define DSLL VR4300_BUILD_OP(DSLL, INV, INFO1(NONE))
#define DSLLV VR4300_BUILD_OP(DSLLV, INV, INFO1(NONE))
#define DSLL32 VR4300_BUILD_OP(DSLL32, DSLL32, INFO1(NEEDRT))
#define DSRA VR4300_BUILD_OP(DSRA, INV, INFO1(NONE))
#define DSRAV VR4300_BUILD_OP(DSRAV, INV, INFO1(NONE))
#define DSRA32 VR4300_BUILD_OP(DSRA32, DSRA32, INFO1(NEEDRT))
#define DSRL VR4300_BUILD_OP(DSRL, INV, INFO1(NONE))
#define DSRLV VR4300_BUILD_OP(DSRLV, INV, INFO1(NONE))
#define DSRL32 VR4300_BUILD_OP(DSRL32, INV, INFO1(NONE))
#define DSUB VR4300_BUILD_OP(DSUB, INV, INFO1(NONE))
#define DSUBU VR4300_BUILD_OP(DSUBU, INV, INFO1(NONE))
#define J VR4300_BUILD_OP(J, J_JAL, INFO1(BRANCH))
#define JAL VR4300_BUILD_OP(JAL, J_JAL, INFO1(BRANCH))
#define JALR VR4300_BUILD_OP(JALR, JALR_JR, INFO2(BRANCH, NEEDRS))
#define JR VR4300_BUILD_OP(JR, JALR_JR, INFO2(BRANCH, NEEDRS))
#define LB VR4300_BUILD_OP(LB, LOAD, INFO1(NEEDRS))
#define LBU VR4300_BUILD_OP(LBU, LOAD, INFO1(NEEDRS))
#define LD VR4300_BUILD_OP(LD, LD, INFO1(NEEDRS))
#define LDC0 VR4300_BUILD_OP(LDC0, INV, INFO1(NONE))
#define LDC1 VR4300_BUILD_OP(LDC1, INV, INFO1(NONE))
#define LDC2 VR4300_BUILD_OP(LDC2, INV, INFO1(NONE))
#define LDL VR4300_BUILD_OP(LDL, INV, INFO1(NONE))
#define LDR VR4300_BUILD_OP(LDR, INV, INFO1(NONE))
#define LH VR4300_BUILD_OP(LH, LOAD, INFO1(NEEDRS))
#define LHU VR4300_BUILD_OP(LHU, LOAD, INFO1(NEEDRS))
#define LL VR4300_BUILD_OP(LL, INV, INFO1(NONE))
#define LLD VR4300_BUILD_OP(LLD, INV, INFO1(NONE))
#define LUI VR4300_BUILD_OP(LUI, LUI, INFO1(NONE))
#define LW VR4300_BUILD_OP(LW, LOAD, INFO1(NEEDRS))
#define LWC0 VR4300_BUILD_OP(LWC0, INV, INFO1(NONE))
#define LWC1 VR4300_BUILD_OP(LWC1, INV, INFO1(NONE))
#define LWC2 VR4300_BUILD_OP(LWC2, INV, INFO1(NONE))
#define LWL VR4300_BUILD_OP(LWL, INV, INFO1(NONE))
#define LWR VR4300_BUILD_OP(LWR, INV, INFO1(NONE))
#define LWU VR4300_BUILD_OP(LWU, LOAD, INFO1(NEEDRS))
#define MFC0 VR4300_BUILD_OP(MFC0, MFCx, INFO1(NONE))
#define MFC1 VR4300_BUILD_OP(MFC1, INV, INFO1(NONE))
#define MFC2 VR4300_BUILD_OP(MFC2, INV, INFO1(NONE))
#define MFHI VR4300_BUILD_OP(MFHI, MFHI_MFLO, INFO1(NONE))
#define MFLO VR4300_BUILD_OP(MFLO, MFHI_MFLO, INFO1(NONE))
#define MTC0 VR4300_BUILD_OP(MTC0, MTCx, INFO1(NEEDRT))
#define MTC1 VR4300_BUILD_OP(MTC1, INV, INFO1(NONE))
#define MTC2 VR4300_BUILD_OP(MTC2, INV, INFO1(NONE))
#define MTHI VR4300_BUILD_OP(MTHI, MTHI_MTLO, INFO1(NEEDRS))
#define MTLO VR4300_BUILD_OP(MTLO, MTHI_MTLO, INFO1(NEEDRS))
#define MULT VR4300_BUILD_OP(MULT, MULT_MULTU, INFO2(NEEDRS, NEEDRT))
#define MULTU VR4300_BUILD_OP(MULTU, MULT_MULTU, INFO2(NEEDRS, NEEDRT))
#define NOR VR4300_BUILD_OP(NOR, INV, INFO1(NONE))
#define OR VR4300_BUILD_OP(OR, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define ORI VR4300_BUILD_OP(ORI, ANDI_ORI_XORI, INFO1(NEEDRS))
#define SB VR4300_BUILD_OP(SB, STORE, INFO2(NEEDRS, NEEDRT))
#define SC VR4300_BUILD_OP(SC, INV, INFO1(NONE))
#define SCD VR4300_BUILD_OP(SCD, INV, INFO1(NONE))
#define SD VR4300_BUILD_OP(SD, SD, INFO2(NEEDRS, NEEDRT))
#define SDC0 VR4300_BUILD_OP(SDC0, INV, INFO1(NONE))
#define SDC1 VR4300_BUILD_OP(SDC1, INV, INFO1(NONE))
#define SDC2 VR4300_BUILD_OP(SDC2, INV, INFO1(NONE))
#define SDL VR4300_BUILD_OP(SDL, INV, INFO1(NONE))
#define SDR VR4300_BUILD_OP(SDR, INV, INFO1(NONE))
#define SH VR4300_BUILD_OP(SH, STORE, INFO2(NEEDRS, NEEDRT))
#define SLL VR4300_BUILD_OP(SLL, SLL, INFO1(NEEDRT))
#define SLLV VR4300_BUILD_OP(SLLV, SLLV, INFO2(NEEDRS, NEEDRT))
#define SLT VR4300_BUILD_OP(SLT, SLT, INFO2(NEEDRS, NEEDRT))
#define SLTI VR4300_BUILD_OP(SLTI, SLTI, INFO1(NEEDRS))
#define SLTIU VR4300_BUILD_OP(SLTIU, SLTIU, INFO1(NEEDRS))
#define SLTU VR4300_BUILD_OP(SLTU, SLTU, INFO2(NEEDRS, NEEDRT))
#define SRA VR4300_BUILD_OP(SRA, SRA, INFO1(NEEDRT))
#define SRAV VR4300_BUILD_OP(SRAV, INV, INFO1(NONE))
#define SRL VR4300_BUILD_OP(SRL, SRL, INFO1(NEEDRT))
#define SRLV VR4300_BUILD_OP(SRLV, SRLV, INFO2(NEEDRS, NEEDRT))
#define SUB VR4300_BUILD_OP(SUB, ADD_SUB, INFO2(NEEDRS, NEEDRT))
#define SUBU VR4300_BUILD_OP(SUBU, ADDU_SUBU, INFO1(NEEDRS))
#define SW VR4300_BUILD_OP(SW, STORE, INFO2(NEEDRS, NEEDRT))
#define SWC0 VR4300_BUILD_OP(SWC0, INV, INFO1(NONE))
#define SWC1 VR4300_BUILD_OP(SWC1, INV, INFO1(NONE))
#define SWC2 VR4300_BUILD_OP(SWC2, INV, INFO1(NONE))
#define SWL VR4300_BUILD_OP(SWL, SWL, INFO2(NEEDRS, NEEDRT))
#define SWR VR4300_BUILD_OP(SWR, SWR, INFO2(NEEDRS, NEEDRT))
#define SYNC VR4300_BUILD_OP(SYNC, INV, INFO1(NONE))
#define SYSCALL VR4300_BUILD_OP(SYSCALL, INV, INFO1(NONE))
#define TEQ VR4300_BUILD_OP(TEQ, INV, INFO1(NONE))
#define TEQI VR4300_BUILD_OP(TEQI, INV, INFO1(NONE))
#define TGE VR4300_BUILD_OP(TGE, INV, INFO1(NONE))
#define TGEI VR4300_BUILD_OP(TGEI, INV, INFO1(NONE))
#define TGEIU VR4300_BUILD_OP(TGEIU, INV, INFO1(NONE))
#define TGEU VR4300_BUILD_OP(TGEU, INV, INFO1(NONE))
#define TLT VR4300_BUILD_OP(TLT, INV, INFO1(NONE))
#define TLTI VR4300_BUILD_OP(TLTI, INV, INFO1(NONE))
#define TLTIU VR4300_BUILD_OP(TLTIU, INV, INFO1(NONE))
#define TLTU VR4300_BUILD_OP(TLTU, INV, INFO1(NONE))
#define TNE VR4300_BUILD_OP(TNE, INV, INFO1(NONE))
#define TNEI VR4300_BUILD_OP(TNEI, INV, INFO1(NONE))
#define XOR VR4300_BUILD_OP(XOR, AND_OR_XOR, INFO2(NEEDRS, NEEDRT))
#define XORI VR4300_BUILD_OP(XORI, ANDI_ORI_XORI, INFO1(NEEDRS))

#endif

