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
#ifndef V8_ARM_REFLECT_ARM_THUMB_H_
#define V8_ARM_REFLECT_ARM_THUMB_H_

#include "arm/constants-arm.h"
#include "arm/reflect-arm-arm.h"

namespace v8 { namespace internal {
namespace thumb16 {
  // IT instruction patching
  bool IsIT(Instr16 instr);
  Condition GetITCondition(Instr16 instr);
  int16_t GetITMask(Instr16 instr);
  Instr16 SetITMask(Instr16 instr, int16_t mask);
  Instr16 AddInstrToITBlock(Instr16 instr, bool take_if_true);

  bool IsBxReg(Instr16 instr);
  bool IsBlxReg(Instr16 instr);
  Register GetBlxRegister(Instr16 instr);

  bool IsNop(Instr16 intr, int type = 0);
} // namespace thumb16

namespace thumb32 {
  // Branches patching
  bool IsBranch(Instr32 instr);
  bool IsConditionalBranch(Instr32 instr);
  bool IsUnconditionalBranch(Instr32 instr);
  bool IsBranchAndLink(Instr32 instr);
  int GetBranchOffset(Instr32 instr);
  Condition GetBranchCondition(Instr32 instr);
  Instr32 SetBranchOffset(Instr32 instr, int offset);
  int CondBranchEncodeOffset(int offset);
  int BranchEncodeOffset(int offset);

  // pc-relative loads patching
  bool IsLdrRegisterImmediate(Instr32 instr);
  bool IsLdrPcImmediateOffset(Instr instr);
  int GetLdrRegisterImmediateOffset(Instr32 instr);
  Instr32 SetLdrRegisterImmediateOffset(Instr32 instr, int offset);

  inline bool IsVldrDRegisterImmediate(Instr32 instr) {
    return arm::IsVldrDRegisterImmediate(instr);
  }
  inline int GetVldrDRegisterImmediateOffset(Instr32 instr) {
    return arm::GetVldrDRegisterImmediateOffset(instr);
  }
  inline Instr32 SetVldrDRegisterImmediateOffset(Instr32 instr, int offset) {
    return arm::SetVldrDRegisterImmediateOffset(instr, offset);
  }
  inline bool IsVldrDPcImmediateOffset(Instr instr) {
    return arm::IsVldrDPcImmediateOffset(instr);
  }

  // movt, movw identity checks
  bool IsMovT(Instr32 instr);
  bool IsMovW(Instr32 instr);
  Instr32 EncodeMovwImmediate(uint32_t imm);
  int GetMovWMovTImmediate(Instr32 instr);
  int SetMovWMovTImmediate(Instr32 instr, int imm);

  // decode components of cmp reg, imm
  bool IsCmpImmediate(Instr32 instr);
  Register GetCmpImmediateRegister(Instr32 instr);
  int GetCmpImmediateRawImmediate(Instr32 instr);
  Instr32 SetCmpImmediateRegister(Instr32 instr, Register reg);
  Instr32 SetCmpImmediateRawImmediate(Instr32 instr, int imm);

  bool IsCmpRegister(Instr32 instr);
  Register GetCmpRegisterRm(Instr32 instr);
  Register GetCmpRegisterRn(Instr32 instr);
  bool IsTstImmediate(Instr32 instr);
  Register GetTstImmediateRegister(Instr32 instr);
}


} }

#endif
