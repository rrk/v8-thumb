// Copyright (c) 1994-2006 Sun Microsystems Inc.
// All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// - Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// - Redistribution in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the
// distribution.
//
// - Neither the name of Sun Microsystems or the names of contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// The original source code covered by the above license above has been
// modified significantly by Google Inc.
// Copyright 2013 the V8 project authors. All rights reserved.
//
#include "v8.h"

#if defined(V8_TARGET_ARCH_ARM)

#include "arm/assembler-arm-inl.h"
#include "arm/reflect-arm-arm.h"

namespace v8 { namespace internal {
namespace arm {
// -----------------------------------------------------------------------------
// Specific instructions, constants, and masks.

// add(sp, sp, 4) instruction (aka Pop())
const Instr kPopInstruction =
    al | PostIndex | 4 | LeaveCC | I | kRegister_sp_Code * B16 |
        kRegister_sp_Code * B12;
// str(r, MemOperand(sp, 4, NegPreIndex), al) instruction (aka push(r))
// register r is not encoded.
const Instr kPushRegPattern =
    al | B26 | 4 | NegPreIndex | kRegister_sp_Code * B16;
// ldr(r, MemOperand(sp, 4, PostIndex), al) instruction (aka pop(r))
// register r is not encoded.
const Instr kPopRegPattern =
    al | B26 | L | 4 | PostIndex | kRegister_sp_Code * B16;
// mov lr, pc
const Instr kMovLrPc = al | MOV | kRegister_pc_Code | kRegister_lr_Code * B12;
// ldr rd, [pc, #offset]
const Instr kLdrPCMask = 15 * B24 | 7 * B20 | 15 * B16;
const Instr kLdrPCPattern = 5 * B24 | L | kRegister_pc_Code * B16;
// vldr dd, [pc, #offset]
const Instr kVldrDPCMask = 15 * B24 | 3 * B20 | 15 * B16 | 15 * B8;
const Instr kVldrDPCPattern = 13 * B24 | L | kRegister_pc_Code * B16 | 11 * B8;
// blxcc rm
const Instr kBlxRegMask =
    15 * B24 | 15 * B20 | 15 * B16 | 15 * B12 | 15 * B8 | 15 * B4;
const Instr kBlxRegPattern =
    B24 | B21 | 15 * B16 | 15 * B12 | 15 * B8 | BLX;
const Instr kBlxIp = al | kBlxRegPattern | ip.code();
const Instr kMovMvnMask = 0x6d * B21 | 0xf * B16;
const Instr kMovMvnPattern = 0xd * B21;
const Instr kMovMvnFlip = B22;
const Instr kMovLeaveCCMask = 0xdff * B16;
const Instr kMovLeaveCCPattern = 0x1a0 * B16;
const Instr kMovwMask = 0xff * B20;
const Instr kMovwPattern = 0x30 * B20;
const Instr kMovwLeaveCCFlip = 0x5 * B21;
const Instr kCmpCmnMask = 0xdd * B20 | 0xf * B12;
const Instr kCmpCmnPattern = 0x15 * B20;
const Instr kCmpCmnFlip = B21;
const Instr kAddSubFlip = 0x6 * B21;
const Instr kAndBicFlip = 0xe * B21;

// A mask for the Rd register for push, pop, ldr, str instructions.
const Instr kLdrRegFpOffsetPattern =
    al | B26 | L | Offset | kRegister_fp_Code * B16;
const Instr kStrRegFpOffsetPattern =
    al | B26 | Offset | kRegister_fp_Code * B16;
const Instr kLdrRegFpNegOffsetPattern =
    al | B26 | L | NegOffset | kRegister_fp_Code * B16;
const Instr kStrRegFpNegOffsetPattern =
    al | B26 | NegOffset | kRegister_fp_Code * B16;
const Instr kLdrStrInstrTypeMask = 0xffff0000;
const Instr kLdrStrInstrArgumentMask = 0x0000ffff;
const Instr kLdrStrOffsetMask = 0x00000fff;
const Instr kBxInstMask = 0x0ffffff0;
const Instr kBxInstPattern = 0x012fff10;

Condition GetCondition(Instr instr) {
  return Instruction::ConditionField(instr);
}

bool IsBranch(Instr instr) {
  return (instr & (B27 | B25)) == (B27 | B25);
}

int GetBranchOffset(Instr instr) {
  ASSERT(IsBranch(instr));
  // Take the jump offset in the lower 24 bits, sign extend it and multiply it
  // with 4 to get the offset in bytes.
  return ((instr & kImm24Mask) << 8) >> 6;
}

bool IsLdrRegisterImmediate(Instr instr) {
  return (instr & (B27 | B26 | B25 | B22 | B20)) == (B26 | B20);
}

bool IsVldrDRegisterImmediate(Instr instr) {
  return (instr & (15 * B24 | 3 * B20 | 15 * B8)) == (13 * B24 | B20 | 11 * B8);
}

int GetLdrRegisterImmediateOffset(Instr instr) {
  ASSERT(IsLdrRegisterImmediate(instr));
  bool positive = (instr & B23) == B23;
  int offset = instr & kOff12Mask;  // Zero extended offset.
  return positive ? offset : -offset;
}

int GetVldrDRegisterImmediateOffset(Instr instr) {
  ASSERT(IsVldrDRegisterImmediate(instr));
  bool positive = (instr & B23) == B23;
  int offset = instr & kOff8Mask;  // Zero extended offset.
  offset <<= 2;
  return positive ? offset : -offset;
}

Instr SetLdrRegisterImmediateOffset(Instr instr, int offset) {
  ASSERT(IsLdrRegisterImmediate(instr));
  bool positive = offset >= 0;
  if (!positive) offset = -offset;
  ASSERT(is_uint12(offset));
  // Set bit indicating whether the offset should be added.
  instr = (instr & ~B23) | (positive ? B23 : 0);
  // Set the actual offset.
  return (instr & ~kOff12Mask) | offset;
}

Instr SetVldrDRegisterImmediateOffset(Instr instr, int offset) {
  ASSERT(IsVldrDRegisterImmediate(instr));
  ASSERT((offset & ~3) == offset);  // Must be 64-bit aligned.
  bool positive = offset >= 0;
  if (!positive) offset = -offset;
  ASSERT(is_uint10(offset));
  // Set bit indicating whether the offset should be added.
  instr = (instr & ~B23) | (positive ? B23 : 0);
  // Set the actual offset. Its bottom 2 bits are zero.
  return (instr & ~kOff8Mask) | (offset >> 2);
}

bool IsStrRegisterImmediate(Instr instr) {
  return (instr & (B27 | B26 | B25 | B22 | B20)) == B26;
}


Instr SetStrRegisterImmediateOffset(Instr instr, int offset) {
  ASSERT(IsStrRegisterImmediate(instr));
  bool positive = offset >= 0;
  if (!positive) offset = -offset;
  ASSERT(is_uint12(offset));
  // Set bit indicating whether the offset should be added.
  instr = (instr & ~B23) | (positive ? B23 : 0);
  // Set the actual offset.
  return (instr & ~kOff12Mask) | offset;
}


bool IsAddRegisterImmediate(Instr instr) {
  return (instr & (B27 | B26 | B25 | B24 | B23 | B22 | B21)) == (B25 | B23);
}


Instr SetAddRegisterImmediateOffset(Instr instr, int offset) {
  ASSERT(IsAddRegisterImmediate(instr));
  ASSERT(offset >= 0);
  ASSERT(is_uint12(offset));
  // Set the offset.
  return (instr & ~kOff12Mask) | offset;
}


Register GetRd(Instr instr) {
  Register reg;
  reg.code_ = Instruction::RdValue(instr);
  return reg;
}


Register GetRn(Instr instr) {
  Register reg;
  reg.code_ = Instruction::RnValue(instr);
  return reg;
}


Register GetRm(Instr instr) {
  Register reg;
  reg.code_ = Instruction::RmValue(instr);
  return reg;
}


bool IsPush(Instr instr) {
  return ((instr & ~kRdMask) == kPushRegPattern);
}


bool IsPop(Instr instr) {
  return ((instr & ~kRdMask) == kPopRegPattern);
}


bool IsStrRegFpOffset(Instr instr) {
  return ((instr & kLdrStrInstrTypeMask) == kStrRegFpOffsetPattern);
}


bool IsLdrRegFpOffset(Instr instr) {
  return ((instr & kLdrStrInstrTypeMask) == kLdrRegFpOffsetPattern);
}


bool IsStrRegFpNegOffset(Instr instr) {
  return ((instr & kLdrStrInstrTypeMask) == kStrRegFpNegOffsetPattern);
}


bool IsLdrRegFpNegOffset(Instr instr) {
  return ((instr & kLdrStrInstrTypeMask) == kLdrRegFpNegOffsetPattern);
}

bool IsLdrPcImmediateOffset(Instr instr) {
  // Check the instruction is indeed a
  // ldr<cond> <Rd>, [pc +/- offset_12].
  return (instr & kLdrPCMask) == kLdrPCPattern;
}

bool IsVldrDPcImmediateOffset(Instr instr) {
  // Check the instruction is indeed a
  // vldr<cond> <Dd>, [pc +/- offset_10].
  return (instr & kVldrDPCMask) == kVldrDPCPattern;
}

bool IsTstImmediate(Instr instr) {
  return (instr & (B27 | B26 | I | kOpCodeMask | S | kRdMask)) ==
      (I | TST | S);
}

bool IsCmpRegister(Instr instr) {
  return (instr & (B27 | B26 | I | kOpCodeMask | S | kRdMask | B4)) ==
      (CMP | S);
}

bool IsCmpImmediate(Instr instr) {
  return (instr & (B27 | B26 | I | kOpCodeMask | S | kRdMask)) ==
      (I | CMP | S);
}

Register GetCmpImmediateRegister(Instr instr) {
  ASSERT(IsCmpImmediate(instr));
  return GetRn(instr);
}

int GetCmpImmediateRawImmediate(Instr instr) {
  ASSERT(IsCmpImmediate(instr));
  return instr & kOff12Mask;
}

bool IsMovT(Instr instr) {
  instr &= ~(((kNumberOfConditions - 1) << 28) |  // Mask off conditions
             ((kNumRegisters-1)*B12) |            // mask out register
             EncodeMovwImmediate(0xFFFF));        // mask out immediate value
  return instr == 0x34*B20;
}


bool IsMovW(Instr instr) {
  instr &= ~(((kNumberOfConditions - 1) << 28) |  // Mask off conditions
             ((kNumRegisters-1)*B12) |            // mask out destination
             EncodeMovwImmediate(0xFFFF));        // mask out immediate value
  return instr == 0x30*B20;
}


bool IsNop(Instr instr, int type) {
  ASSERT(0 <= type && type <= 14);  // mov pc, pc isn't a nop.
  // Check for mov rx, rx where x = type.
  return instr == (al | 13*B21 | type*B12 | type);
}

Instr EncodeMovwImmediate(uint32_t immediate) {
  ASSERT(immediate < 0x10000);
  return ((immediate & 0xf000) << 4) | (immediate & 0xfff);
}

bool IsBxReg(Instr instr) {
  return (instr & kBxInstMask) == kBxInstPattern;
}

bool IsBlxReg(Instr instr) {
  return (instr & kBlxRegMask) == kBlxRegPattern;
}

bool IsMovLrPc(Instr instr) {
  return instr == kMovLrPc;
}

} // namespace arm
} }

#endif
