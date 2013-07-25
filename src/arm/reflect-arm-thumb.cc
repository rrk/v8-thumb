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
#include "arm/reflect-arm-thumb.h"

namespace v8 { namespace internal {
namespace thumb16 {

bool IsIT(Instr16 instr) {
  using namespace thumb16::misc;
  return Instr16(instr & IT_MASK) == Instr16(IT);
}

Condition GetITCondition(Instr16 instr) {
  using namespace thumb16::misc;
  ASSERT(IsIT(instr));
  return CodeToCondition((instr & IT_COND_MASK) >> 4);
}

int16_t GetITMask(Instr16 instr) {
  using namespace thumb16::misc;
  ASSERT(IsIT(instr));
  return instr & IT_MASK_MASK;
}

Instr16 SetITMask(Instr16 instr, int16_t mask) {
  using namespace thumb16::misc;
  ASSERT(IsIT(instr));
  ASSERT(is_uint4(mask));
  return (instr & ~IT_MASK_MASK) | mask;
}

Instr16 AddInstrToITBlock(Instr16 instr, bool take_if_true) {
  using namespace thumb16::misc;
  bool cc_bit = instr & IT_COND_LB; // lower bit of the condition
  // Find the lowest bit in the mask
  int16_t mask = GetITMask(instr);
  if (mask) {
    int cursor_bit = 0;
    while (cursor_bit < 4 && (mask & (1 << cursor_bit)) == 0) cursor_bit++;
    // if cursor_bit == 0, that means that the block is about to overflow
    ASSERT(cursor_bit > 0);
    // control bit is the lower bit of the condition if
    // the branch is taken, inverse of that if not.
    bool next_bit = !(take_if_true ^ cc_bit);
    // set the value of bit in lb position to next_bit value
    mask = (mask & ~(1 << cursor_bit)) | (next_bit << cursor_bit);
    ASSERT((mask & (1 << (cursor_bit - 1))) == 0);
    mask |= 1 << (cursor_bit - 1);
  } else {
    // The first bit will have to be always set to 1
    ASSERT(take_if_true);
    mask |= B3;
  }
  return SetITMask(instr, mask);
}

bool IsBxReg(Instr16 instr) {
  using namespace thumb16::spec_data;
  return Instr16(instr & BRANCH_MASK) == Instr16(BX);
}

bool IsBlxReg(Instr16 instr) {
  using namespace thumb16::spec_data;
  return Instr16(instr & BRANCH_MASK) == Instr16(BLX);
}

Register GetBlxRegister(Instr16 instr) {
  using namespace thumb16::spec_data;
  ASSERT(IsBlxReg(instr));
  return Register::from_code((instr >> BRANCH_REG_SHIFT) & 0xf);
}

bool IsNop(Instr16 instr, int type) {
  using namespace thumb16::spec_data;
  if (Instr16(instr & MOV_MASK) == Instr16(MOV)) {
    int src_code = (instr & MOV_SRC_MASK) >> MOV_SRC_SHIFT;
    int dst_code = (instr & MOV_DST_MASK);
    dst_code = (dst_code >> (MOV_DST_SHIFT1 - MOV_DST_POS1)) | (dst_code & MOV_DST_MASK2);
    return src_code == type && src_code == dst_code;
  }
  return false;
}

} // namespace thumb16

namespace thumb32 {


bool IsBranch(Instr32 instr) {
  using namespace thumb32::control;
  Instr32 x = instr & BRANCH_MASK;
  return x == Instr32(BL) || x == Instr32(BLX) || x == Instr32(COND_B) || x == Instr32(AL_B);
}

bool IsConditionalBranch(Instr32 instr) {
  using namespace thumb32::control;
  return Instr32(instr & BRANCH_MASK) == Instr32(COND_B);
}

bool IsUnconditionalBranch(Instr32 instr) {
  using namespace thumb32::control;
  return Instr32(instr & BRANCH_MASK) == Instr32(AL_B);
}

bool IsBranchAndLink(Instr32 instr) {
  using namespace thumb32::control;
  return Instr32(instr & BRANCH_MASK) == Instr32(BL);
}

int GetBranchOffset(Instr32 instr) {
  using namespace thumb32::control;
  ASSERT(IsBranch(instr));
  const bool s = instr & H10;
  const bool j1 = instr & L13;
  const bool j2 = instr & L11;
  const int imm11 = instr & 0x7FF;
  if (IsConditionalBranch(instr)) {
    const int imm6 = (instr >> 16) & 0x3F;
    int imm = imm11 | imm6 * B11 | j1 * B17 | j2 * B18 | s * B19;
    imm = (imm << 12) >> 11;
    return imm;
  } else {
    const int imm10 = (instr >> 16) & 0x3FF;
    const bool i1 = !(j1 ^ s);
    const bool i2 = !(j2 ^ s);
    int imm = imm11 | imm10 * B11 | i2 * B21 | i1 * B22 | s * B23;
    imm = (imm << 8) >> 7;
    return imm;
  }
}

Condition GetBranchCondition(Instr32 instr) {
  ASSERT(IsBranch(instr));
  if (IsConditionalBranch(instr)) {
    return CodeToCondition((instr >> 22) & 0xF);
  }
  return al;
}

int CondBranchEncodeOffset(int offset) {
  ASSERT(is_intn(offset, 20));
  // Split into S:J2:J1:imm6:imm11
  const int imm11 = offset & 0x7FF;
  const int imm6  = (offset >> 11) & 0x3F;
  const bool j1   = (offset >> 17) & 0x1;
  const bool j2   = (offset >> 18) & 0x1;
  const bool s    = (offset >> 19) & 0x1;
  return s * H10 | imm6 * H0 | j1 * L13 | j2 * L11 | imm11 * L0;
}

int BranchEncodeOffset(int offset) {
  ASSERT(is_intn(offset, 24));
  // Split into S:I1:I2:imm10:imm11
  const int imm11 = offset & 0x7FF;
  const int imm10 = (offset >> 11) & 0x3FF;
  const bool i2   = (offset >> 21) & 0x1;
  const bool i1   = (offset >> 22) & 0x1;
  const bool s    = (offset >> 23) & 0x1;
  // Now, the encoding wants J1 and J2 bits computed as:
  // J = (not I) xor S
  const bool j1 = !i1 ^ s;
  const bool j2 = !i2 ^ s;
  return s * H10 | imm10 * H0 | j1 * L13 | j2 * L11 | imm11 * L0;
}


Instr32 SetBranchOffset(Instr32 instr, int offset) {
  using namespace thumb32::control;
  ASSERT(IsBranch(instr));
  // in thumb jumps are at least 2-byte aligned
  ASSERT((offset & 1) == 0);
  if (IsConditionalBranch(instr)) {
    instr &= BRANCH_MASK | H6 | H7 | H8 | H9;
    instr |= CondBranchEncodeOffset(offset >> 1);
    ASSERT(GetBranchOffset(instr) == offset);
    return instr;
  } else {
    // BLX expects 4-byte aligned target
    ASSERT((instr & BRANCH_MASK) != BLX || (offset & 3) == 0);
    instr &= BRANCH_MASK;
    instr |= BranchEncodeOffset(offset >> 1);
    ASSERT(GetBranchOffset(instr) == offset);
    return instr;
  }
}

bool IsLdrRegisterImmediate(Instr32 instr) {
  using namespace thumb32::mem;
  instr &= LDR_MASK;
  return instr == Instr32(LDR1) || instr == Instr32(LDR2);
}

bool IsLdrPcImmediateOffset(Instr instr) {
  if (IsLdrRegisterImmediate(instr)) {
    Register rn = Register::from_code((instr >> 16) & 0xf);
    return rn.is(pc);
  }
  return false;
}

int GetLdrRegisterImmediateOffset(Instr32 instr) {
  ASSERT(IsLdrRegisterImmediate(instr));
  using namespace thumb32::mem;
  bool _U = true;
  Instr32 m = instr & LDR_MASK;

  Register rn = Register::from_code((instr >> 16) & 0xf);
  if (m == Instr32(LDR1) || rn.is(pc)) {
    if (rn.is(pc)) {
      _U = instr & H7;
    }
    int offset = instr & 0xfff;
    return _U ? offset : -offset;
  } else if (m == Instr32(LDR2)) {
    _U = instr & L9;
    int offset = instr & 0xff;
    return _U ? offset : -offset;
  }
  UNREACHABLE();
  return 0;
}

Instr32 SetLdrRegisterImmediateOffset(Instr32 instr, int offset) {
  ASSERT(IsLdrRegisterImmediate(instr));
  using namespace thumb32::mem;
  bool _U = true;
  Instr32 m = instr & LDR_MASK;
  Register rn = Register::from_code((instr >> 16) & 0xf);
  if (m == Instr32(LDR1) || rn.is(pc)) {
    if (rn.is(pc) && offset < 0) {
      _U = false;
      offset = -offset;
    } else {
      ASSERT(offset > 0);
    }
    ASSERT(is_uintn(offset, 12));
    return (instr & ~(H7 | 0xfff)) | (_U * H7) | offset;
  } else if (m == Instr32(LDR2)) {
    if (offset < 0) {
      offset = -offset;
      _U = false;
    }
    ASSERT(is_uintn(offset, 8));
    return (instr & ~(L9 | 0xff)) | (_U * L9) | offset;
  }
  UNREACHABLE();
  return 0;
}


bool IsMovT(Instr32 instr) {
  using namespace thumb32::plain_imm;
  return Instr32(instr & MOV_MASK) == Instr32(MOVT);
}

bool IsMovW(Instr32 instr) {
  using namespace thumb32::plain_imm;
  return Instr32(instr & MOV_MASK) == Instr32(MOVW);
}

Instr32 EncodeMovwImmediate(uint32_t imm) {
  ASSERT(is_uint16(imm));
  // Split the immediate into the imm4:imm1:imm3:imm8 form
  const int imm4 = (imm & 0xffff) >> 12;
  const int imm1 = (imm & 0xfff)  >> 11;
  const int imm3 = (imm & 0x7ff)  >> 8;
  const int imm8 = (imm & 0xff);
  return imm1 * H10 | imm4 * H0 | imm3 * L12 | imm8 * L0;
}

int GetMovWMovTImmediate(Instr32 instr) {
  ASSERT(IsMovW(instr) || IsMovT(instr));
  const bool imm1 = instr & H10;
  const int imm4 = (instr >> 16) & 0xf;
  const int imm3 = (instr >> 12) & 0x7;
  const int imm8 = instr & 0xff;
  return imm8 | imm3 * B8 | imm4 * B11 | imm1 * B15;
}

Instr32 SetMovWMovTImmediate(Instr32 instr, int imm) {
  using namespace thumb32::plain_imm;
  ASSERT(IsMovW(instr) || IsMovT(instr));
  instr &= MOV_IMM_MASK;
  instr |= EncodeMovwImmediate(imm);
  ASSERT(GetMovWMovTImmediate(instr) == imm);
  return instr;
}

bool IsCmpImmediate(Instr32 instr) {
  return Instr32(instr & mod_imm::CMP_MASK) == Instr32(mod_imm::CMP);
}

Register GetCmpImmediateRegister(Instr32 instr) {
  ASSERT(IsCmpImmediate(instr));
  return Register::from_code((instr >> mod_imm::CMP_REG_SHIFT) & 0xf);
}

Instr32 SetCmpImmediateRegister(Instr32 instr, Register reg) {
  ASSERT(IsCmpImmediate(instr));
  return (instr & ~mod_imm::CMP_REG_MASK) | reg.code() * H0;
}

Instr32 SetCmpImmediateRawImmediate(Instr32 instr, int imm) {
  ASSERT(IsCmpImmediate(instr));
  ASSERT(is_uint12(imm));
  const int imm1 = (imm >> 11) & 0x1;
  const int imm3 = (imm >> 8) & 0x7;
  const int imm8 = imm & 0xff;
  return (instr & ~mod_imm::CMP_IMM_MASK) | imm1 * H10 | imm3 * L12 | imm8 * L0;
}

int GetCmpImmediateRawImmediate(Instr32 instr) {
  ASSERT(IsCmpImmediate(instr));
  const bool imm1 = instr & H10;
  const int imm3 = (instr >> 12) & 0x7;
  const int imm8 = instr & 0xff;
  return imm8 | (imm3 << 8) | (imm1 << 11);
}

bool IsCmpRegister(Instr32 instr) {
  return Instr32(instr & shifted_reg::CMP_MASK) == Instr32(shifted_reg::CMP);
}

Register GetCmpRegisterRm(Instr32 instr) {
  ASSERT(IsCmpRegister(instr));
  return Register::from_code((instr >> shifted_reg::CMP_RM_SHIFT) & 0xf);
}

Register GetCmpRegisterRn(Instr32 instr) {
  ASSERT(IsCmpRegister(instr));
  return Register::from_code((instr >> shifted_reg::CMP_RN_SHIFT) & 0xf);
}

bool IsTstImmediate(Instr32 instr) {
  return Instr32(instr & mod_imm::TST_MASK) == Instr32(mod_imm::TST);
}

Register GetTstImmediateRegister(Instr32 instr) {
  ASSERT(IsTstImmediate(instr));
  return Register::from_code((instr >> mod_imm::TST_REG_SHIFT) & 0xf);
}

} // namespace thumb32

} }

#endif

