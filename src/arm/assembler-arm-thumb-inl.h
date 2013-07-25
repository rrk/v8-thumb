
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

#ifndef V8_ARM_ASSEMBLER_ARM_THUMB_INL_H_
#define V8_ARM_ASSEMBLER_ARM_THUMB_INL_H_

namespace v8 {
namespace internal {

void Assembler::emit32(Instr32 x, Condition cond) {
  CheckBuffer();
  if (cond != kSpecialCondition) {
    cond32(cond, true);
  }
  using namespace thumb32;
#ifdef DEBUG
  const int prefix = x & T32PrefixMask;
  ASSERT(prefix == T32Prefix01 || prefix == T32Prefix10 || prefix == T32Prefix11);
#endif
  // we assume that we're in little-endian mode, so swap the halfwords before store
  *reinterpret_cast<Instr32*>(pc_) = swap32(x);
  pc_ += sizeof(Instr32);
}

// emit 16bit thumb instruction
void Assembler::emit16(Instr16 x, SBit s, Condition c) {
  CheckBuffer();
  if (c != kSpecialCondition) {
    if (s == SetCC) {
      ASSERT(c == al);
      flush_cond();
    } else {
      // s == LeaveCC
      cond(c, true);
    }
  }
  *reinterpret_cast<Instr16*>(pc_) = x;
  pc_ += sizeof(Instr16);
}

void Assembler::emit16(Instr16 x, Condition cond) {
  // This emitter is for kind of instructions that has the behavior (wrt IT blocks)
  // analogous to 32bit instructions. That is they are conditionally executed and may
  // set flags, but there is no flag setting suppression if they are in IT block.
  CheckBuffer();
  if (cond != kSpecialCondition) {
    cond32(cond, true);
  }
  *reinterpret_cast<Instr16*>(pc_) = x;
  pc_ += sizeof(Instr16);
}

void Assembler::flush_cond() {
  using namespace thumb16;
  if (it_instr_pos_ != -1) {
    ASSERT(IsIT(instr16_at(it_instr_pos_)));
    ASSERT(it_size_ > 0); // Otherwise it would be an illegal instruction
    it_instr_pos_ = -1;
    it_size_ = 0;
  }
}

// Patch the existing IT instruction, or emit a new one
void Assembler::cond(Condition c, bool expand_block) {
  using namespace thumb16;
  if (it_instr_pos_ != -1) { // If there's existing open IT block
    Instr16 instr = instr16_at(it_instr_pos_);
    Condition it_cond = GetITCondition(instr);
    ASSERT(IsIT(instr));
    ASSERT(it_size_ < 4);
    if (it_cond == c) {
      // Is the condition the same? Can we just continue the block?
      if (expand_block) {
        instr16_at_put(it_instr_pos_, AddInstrToITBlock(instr, true));
        it_size_++;
      }
    } else if (c != al && it_cond == NegateCondition(c)) {
      // Is the condition reverse? We can do the "else" instruction then.
      if (expand_block) {
        instr16_at_put(it_instr_pos_, AddInstrToITBlock(instr, false));
        it_size_++;
      }
    } else {
      // Other condition. Close the block and start a new one.
      flush_cond();
      cond(c, expand_block);
    }
    // Last instruction in the block, close it.
    if (it_size_ >= 4) {
      flush_cond();
    }
  } else {
    // Open a new IT block.
    it_instr_pos_ = pc_offset();
    it_size_ = 0;
    it(c, 0);
    if (expand_block) {
      // First instruction is always in the "true" arm of the if.
      instr16_at_put(it_instr_pos_, AddInstrToITBlock(instr16_at(it_instr_pos_), true));
      it_size_++;
    }
  }
  ASSERT(it_size_ < 4); // Invariant
}

#define AGGRESSIVELY_EXPAND_IT_BLOCKS 1

void Assembler::cond32(Condition c, bool expand_block) {
  using namespace thumb16;
  if (c == al) {
    if (it_instr_pos_ != -1) {
#if AGGRESSIVELY_EXPAND_IT_BLOCKS
      if (GetITCondition(instr16_at(it_instr_pos_)) == al) {
        cond(c, expand_block);
      } else {
        flush_cond();
      }
#else
      flush_cond();
#endif
    }
  } else {
    cond(c, expand_block);
  }
}

bool RelocInfo::IsPatchedReturnSequence() {
  using namespace thumb16;
  using namespace thumb32;
  // A patched return sequence is:
  //  ldr ip, [pc, #0]
  //  blx ip
  return IsLdrPcImmediateOffset(Assembler::instr32_at(pc_)) &&
         IsBlxReg(Assembler::instr16_at(pc_ + Assembler::kInstrSize * 2));
}

Address Assembler::target_address_from_return_address(Address pc) {
  using namespace thumb32;
  // Strip the lower bit
  pc = reinterpret_cast<Address>(reinterpret_cast<intptr_t>(pc) & ~1);
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
  Address candidate = pc - 3 * Assembler::kInstrSize;
  Instr32 candidate_instr(Assembler::instr32_at(candidate));
  if (IsLdrPcImmediateOffset(candidate_instr)) {
    return candidate;
  }
  candidate = pc - 5 * Assembler::kInstrSize;
  ASSERT(IsMovW(Assembler::instr32_at(candidate)) &&
         IsMovT(Assembler::instr32_at(candidate + kInstrSize*2)));
  return candidate;
}

Address Assembler::return_address_from_call_start(Address pc) {
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  using namespace thumb32;
  if (IsLdrPcImmediateOffset(Assembler::instr32_at(pc))) {
    return pc + kInstrSize * 3;
  } else {
    ASSERT(IsMovW(Assembler::instr32_at(pc)));
    ASSERT(IsMovT(Assembler::instr32_at(pc + kInstrSize)));
    return pc + kInstrSize * 5;
  }
}

Address Assembler::target_pointer_at(Address pc) {
  using namespace thumb32;
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  Instr32 first_instr = Assembler::instr32_at(pc);
  Instr32 second_instr = Assembler::instr32_at(pc + 2 * kInstrSize);
  if (IsMovW(first_instr)) {
    ASSERT(IsMovT(second_instr));
    return reinterpret_cast<Address>(
        (GetMovWMovTImmediate(second_instr) << 16) |
        GetMovWMovTImmediate(first_instr));
  }
  return Memory::Address_at(target_pointer_address_at(pc));
}

void Assembler::set_target_pointer_at(Address pc, Address target) {
  using namespace thumb32;
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  if (IsMovW(Assembler::instr32_at(pc))) {
    Instr32 first_instr = Assembler::instr32_at(pc);
    Instr32 second_instr = Assembler::instr32_at(pc + 2 * kInstrSize);
    ASSERT(IsMovT(second_instr));
    uint32_t immediate = reinterpret_cast<uint32_t>(target);
    SetMovWMovTImmediate(first_instr, immediate & 0xffff);
    SetMovWMovTImmediate(second_instr, immediate >> 16);
    CPU::FlushICache(pc, 4 * kInstrSize);
  } else {
    ASSERT(IsLdrPcImmediateOffset(Assembler::instr32_at(pc)));
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

Address Assembler::target_pointer_address_at(Address pc) {
  using namespace thumb16;
  using namespace thumb32;
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  Address target_pc = pc;
  Instr16 first_instr = Assembler::instr16_at(target_pc);
  // If we have a bx instruction, the instruction before the bx is
  // what we need to patch.
  if (IsBxReg(first_instr)) {
    target_pc -= kInstrSize;
  }

  // If we have a blx instruction, the instruction before it is
  // what needs to be patched.
  if (IsBlxReg(first_instr)) {
    target_pc -= kInstrSize;
  }

  Instr32 instr = Assembler::instr32_at(target_pc);
  ASSERT(IsLdrPcImmediateOffset(instr));
  int offset = GetLdrRegisterImmediateOffset(instr);
  ASSERT(offset >= -4);
  return RoundDown(target_pc, 4) + offset + kPcLoadDelta;
}

Address Assembler::target_address_at(Address pc) {
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  return CPU::DecodePcAddress(target_pointer_at(pc));
}

void Assembler::set_target_address_at(Address pc, Address target) {
  ASSERT((reinterpret_cast<intptr_t>(pc) & 1) == 0);
  set_target_pointer_at(pc, CPU::EncodePcAddress(target));
}

} } // namespace v8::internal

#endif // V8_ARM_ASSEMBLER_ARM_THUMB_INL_H_
