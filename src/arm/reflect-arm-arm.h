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
#ifndef V8_ARM_REFLECT_ARM_ARM_H_
#define V8_ARM_REFLECT_ARM_ARM_H_

#include "arm/constants-arm.h"

namespace v8 { namespace internal {
// misc instruction reflection support for classic arm
namespace arm {
  Condition GetCondition(Instr instr);
  bool IsBranch(Instr instr);
  int GetBranchOffset(Instr instr);
  bool IsLdrRegisterImmediate(Instr instr);
  bool IsVldrDRegisterImmediate(Instr instr);
  int GetLdrRegisterImmediateOffset(Instr instr);
  int GetVldrDRegisterImmediateOffset(Instr instr);
  Instr SetLdrRegisterImmediateOffset(Instr instr, int offset);
  Instr SetVldrDRegisterImmediateOffset(Instr instr, int offset);
  bool IsStrRegisterImmediate(Instr instr);
  Instr SetStrRegisterImmediateOffset(Instr instr, int offset);
  bool IsAddRegisterImmediate(Instr instr);
  Instr SetAddRegisterImmediateOffset(Instr instr, int offset);
  Register GetRd(Instr instr);
  Register GetRn(Instr instr);
  Register GetRm(Instr instr);
  bool IsPush(Instr instr);
  bool IsPop(Instr instr);
  bool IsStrRegFpOffset(Instr instr);
  bool IsLdrRegFpOffset(Instr instr);
  bool IsStrRegFpNegOffset(Instr instr);
  bool IsLdrRegFpNegOffset(Instr instr);
  bool IsLdrPcImmediateOffset(Instr instr);
  bool IsVldrDPcImmediateOffset(Instr instr);
  bool IsTstImmediate(Instr instr);
  bool IsCmpRegister(Instr instr);
  bool IsCmpImmediate(Instr instr);
  Register GetCmpImmediateRegister(Instr instr);
  int GetCmpImmediateRawImmediate(Instr instr);
  bool IsNop(Instr instr, int type = 0);
  bool IsMovT(Instr instr);
  bool IsMovW(Instr instr);
  Instr EncodeMovwImmediate(uint32_t immediate);
  bool IsBxReg(Instr instr);
  bool IsBlxReg(Instr instr);
  bool IsMovLrPc(Instr instr);
} } }

#endif
