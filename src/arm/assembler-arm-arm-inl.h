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

// The original source code covered by the above license above has been modified
// significantly by Google Inc.
// Copyright 2013 the V8 project authors. All rights reserved.

#ifndef V8_ARM_ASSEMBLER_ARM_ARM_INL_H_
#define V8_ARM_ASSEMBLER_ARM_ARM_INL_H_


namespace v8 {
namespace internal {

bool RelocInfo::IsPatchedReturnSequence() {
  using namespace arm;
  Instr current_instr = Assembler::instr_at(pc_);
  Instr next_instr = Assembler::instr_at(pc_ + Assembler::kInstrSize);
  // A patched return sequence is:
  //  ldr ip, [pc, #0]
  //  blx ip
  return IsLdrPcImmediateOffset(current_instr) &&
         IsBlxReg(next_instr);
}

Address Assembler::target_address_from_return_address(Address pc) {
  using namespace arm;
  // Returns the address of the call target from the return address that will
  // be returned to after a call.
  // Call sequence on V7 or later is :
  //  movw  ip, #... @ call address low 16
  //  movt  ip, #... @ call address high 16
  //  blx   ip
  //                      @ return address
  // Or pre-V7 or cases that need frequent patching:
  //  ldr   ip, [pc, #...] @ call address
  //  blx   ip
  //                      @ return address
  Address candidate = pc - 2 * Assembler::kInstrSize;
  Instr candidate_instr(Memory::int32_at(candidate));
  if (IsLdrPcImmediateOffset(candidate_instr)) {
    return candidate;
  }
  candidate = pc - 3 * Assembler::kInstrSize;
  ASSERT(IsMovW(Memory::int32_at(candidate)) &&
         IsMovT(Memory::int32_at(candidate + kInstrSize)));
  return candidate;
}

Address Assembler::target_pointer_at(Address pc) {
  using namespace arm;
  if (IsMovW(Memory::int32_at(pc))) {
    ASSERT(IsMovT(Memory::int32_at(pc + kInstrSize)));
    Instruction* instr = Instruction::At(pc);
    Instruction* next_instr = Instruction::At(pc + kInstrSize);
    return reinterpret_cast<Address>(
        (next_instr->ImmedMovwMovtValue() << 16) |
        instr->ImmedMovwMovtValue());
  }
  return Memory::Address_at(target_pointer_address_at(pc));
}

void Assembler::set_target_pointer_at(Address pc, Address target) {
  using namespace arm;
  if (IsMovW(Memory::int32_at(pc))) {
    ASSERT(IsMovT(Memory::int32_at(pc + kInstrSize)));
    uint32_t* instr_ptr = reinterpret_cast<uint32_t*>(pc);
    uint32_t immediate = reinterpret_cast<uint32_t>(target);
    uint32_t intermediate = instr_ptr[0];
    intermediate &= ~EncodeMovwImmediate(0xFFFF);
    intermediate |= EncodeMovwImmediate(immediate & 0xFFFF);
    instr_ptr[0] = intermediate;
    intermediate = instr_ptr[1];
    intermediate &= ~EncodeMovwImmediate(0xFFFF);
    intermediate |= EncodeMovwImmediate(immediate >> 16);
    instr_ptr[1] = intermediate;
    ASSERT(IsMovW(Memory::int32_at(pc)));
    ASSERT(IsMovT(Memory::int32_at(pc + kInstrSize)));
    CPU::FlushICache(pc, 2 * kInstrSize);
  } else {
    ASSERT(IsLdrPcImmediateOffset(Memory::int32_at(pc)));
    Memory::Address_at(target_pointer_address_at(pc)) = target;
    // Intuitively, we would think it is necessary to always flush the
    // instruction cache after patching a target address in the code as follows:
    //   CPU::FlushICache(pc, sizeof(target));
    // However, on ARM, no instruction is actually patched in the case
    // of embedded constants of the form:
    // ldr   ip, [pc, #...]
    // since the instruction accessing this address in the constant pool remains
    // unchanged.
  }
}

Address Assembler::return_address_from_call_start(Address pc) {
  using namespace arm;
  if (IsLdrPcImmediateOffset(Memory::int32_at(pc))) {
    return pc + kInstrSize * 2;
  } else {
    ASSERT(IsMovW(Memory::int32_at(pc)));
    ASSERT(IsMovT(Memory::int32_at(pc + kInstrSize)));
    return pc + kInstrSize * 3;
  }
}

Address Assembler::target_pointer_address_at(Address pc) {
  using namespace arm;
  Address target_pc = pc;
  Instr instr = Memory::int32_at(target_pc);
  // If we have a bx instruction, the instruction before the bx is
  // what we need to patch.
  if (IsBxReg(instr)) {
    target_pc -= kInstrSize;
    instr = Memory::int32_at(target_pc);
  }

  // If we have a blx instruction, the instruction before it is
  // what needs to be patched.
  if (IsBlxReg(instr)) {
    target_pc -= kInstrSize;
    instr = Memory::int32_at(target_pc);
  }

  ASSERT(IsLdrPcImmediateOffset(instr));
  int offset = GetLdrRegisterImmediateOffset(instr);
  ASSERT(offset >= -4);
  return target_pc + offset + kPcLoadDelta;
}

Address Assembler::target_address_at(Address pc) {
  return target_pointer_at(pc);
}


void Assembler::set_target_address_at(Address pc, Address target) {
  set_target_pointer_at(pc, target);
}

} } // namespace v8::internal

#endif // V8_ARM_ASSEMBLER_ARM_ARM_INL_H_
