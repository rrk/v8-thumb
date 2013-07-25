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

#include "v8.h"

#if defined(V8_TARGET_ARCH_ARM)

#include "arm/assembler-arm-inl.h"
#include "serialize.h"

namespace v8 {
namespace internal {

void Assembler::fold_rm(Register r, const Operand& x, Condition cond) {
  if (!x.rm().is_valid()) {
    // Immediate
    move_32_bit_immediate(cond, r, LeaveCC, x);
  } else if (x.rs().is_valid()) {
    // Shift by reg
    thumb_shift(x.shift_op(), r, x.rm(), Operand(x.rs()), LeaveCC, cond);
  } else {
    // Default
    mov(r, x, LeaveCC, cond);
  }
}


// Data-processing instructions.
bool Assembler::addrmod1_16_1(const ThumbAddrMod1Options& o,
                              Register dst, Register src1, const Operand& src2,
                              SBit s, Condition cond) {
  using namespace thumb16;
  Instr16 instr;
  bool it_effect;
  // Special 16-bit instructions (reg-to-reg) that allow pc and sp, they do not
  // however set flags (except CMP) and behave the same way as 32bit instructions.
  // We call the special 2-arg version of emit16 to convey that to the emitter.
  if (match(&instr, &it_effect, o.spec_data_op, dst, src1, src2, s)) {
    ASSERT(it_effect == false);
    emit16(instr, cond);
    return true;
  }

  return false;
}

bool Assembler::addrmod1_16_2(const ThumbAddrMod1Options& o,
                              Register dst, Register src1, const Operand& src2,
                              SBit s, Condition cond) {
  // There are different kinds of instructions with unique constraints,
  // especially in this category, like the immediate can be 8 bits,
  // but then the Rd and Rn should be the same, or the immediate can be 3 bits,
  // but Rd and Rn can different, and so on and on. We recieve the possible
  // forms to look at in the data_proc1_op[] array (up to 3 elements).
  // We just let match() check the constraints in a loop.
  using namespace thumb16::data_proc1;
  Instr16 instr;
  bool it_effect;
  for (int i = 0; o.data_proc1_op[i] != NONE; i++) {
    ASSERT(static_cast<size_t>(i) < sizeof(o.data_proc1_op) / sizeof(Opcode));
    if (match(&instr, &it_effect, o.data_proc1_op[i], dst, src1, src2, s, cond)) {
      if (it_effect) {
        // IT block suppresses flags
        emit16(instr, s, cond);
      } else {
        emit16(instr, cond);
      }
      return true;
    }
  }
  return false;
}

bool Assembler::addrmod1_16_3(const ThumbAddrMod1Options& o,
                              Register dst, Register src1, const Operand& src2,
                              SBit s, Condition cond) {
  using namespace thumb16::data_proc2;
  Instr16 instr;
  bool it_effect;
  if (match(&instr, &it_effect, o.data_proc2_op, dst, src1, src2, s, cond)) {
    if (it_effect) {
      // IT block suppresses flags
      emit16(instr, s, cond);
    } else {
      emit16(instr, cond);
    }
    return true;
  }
  return false;
}

bool Assembler::addrmod1_32(const ThumbAddrMod1Options& o,
                            Register dst, Register src1, const Operand& src2,
                            SBit s, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, o.plain_imm_op, dst, src1, src2, s) ||
      match(&instr, o.mod_imm_op, dst, src1, src2, s) ||
      match(&instr, o.shifted_reg_op, dst, src1, src2, s)) {
    emit32(instr, cond);
    return true;
  }
  return false;
}

bool Assembler::addrmod1(const ThumbAddrMod1Options& o,
                         Register dst, Register src1, const Operand& src2,
                         SBit s, Condition cond, bool ret_on_failure) {
  if (src2.rm().is_valid() || !src2.must_output_reloc_info(this)) {
    bool done = false;
    if (predictable_code_size()) {
      // The instructions that come out of addrmod1_16_1 all have full register file access,
      // and therefore are stable.
      done = done || addrmod1_16_1(o, dst, src1, src2, s, cond);
      // Next, try the 32bit instructions rightaway.
      done = done || addrmod1_32(o, dst, src1, src2, s, cond);
    } else {
      // Normal selection order - prefer smaller instructions
      // Try to select a 16-bit instruction
      done = done || addrmod1_16_1(o, dst, src1, src2, s, cond);
      // We generate the following forms only if SetCC is requested. IT AL blocks seem to
      // cause significant slowdowns.
      if (s == SetCC || FLAG_enable_size_opt) {
        done = done || addrmod1_16_2(o, dst, src1, src2, s, cond);
        done = done || addrmod1_16_3(o, dst, src1, src2, s, cond);
      }
      // Ok, try to select a 32-bit instruction, there are 3 possible forms that
      // ALU operations may (must, if src2 is a register) fit.
      done = done || addrmod1_32(o, dst, src1, src2, s, cond);
    }

    // If we modified pc, we should close the IT block
    if (done) {
      if (dst.is(pc)) {
        flush_cond();
      }
      return true;
    }

    if (src2.is_reg() && ret_on_failure) {
      return false;
    }

    ASSERT(!src2.is_reg());
  }

  if (o.is_move && s == LeaveCC) {
    fold_rm(dst, src2, cond);
    return true;
  }

  // Spill fold big/complex src2 into a temporary and try to emit again.
  if (src1.is(ip) && ret_on_failure) {
    return false;
  }
  ASSERT(!src1.is(ip));
  fold_rm(ip, src2, cond);
  return addrmod1(o, dst, src1, Operand(ip), s, cond, ret_on_failure);
}

void Assembler::thumb_shift(ShiftOp shift_op, Register dst, Register src1, const Operand& x, SBit s, Condition cond) {
  using namespace thumb16;
  using namespace thumb32;
  Instr16 instr16;
  bool it_effect;
  if (!predictable_code_size() && match(&instr16, &it_effect, shift_op, dst, src1, x, s, cond)) {
    ASSERT(it_effect == true);
    emit16(instr16, s, cond);
  } else {
    Instr32 instr32;
    if (match(&instr32, shift_op, dst, src1, x, s)) {
      emit32(instr32, cond);
    } else UNIMPLEMENTED();
  }
}

// Multiply instructions.
void Assembler::thumb_mul(thumb16::data_proc2::Opcode op1,
                          thumb32::mul::Opcode op2,
                          Register dst, Register src1,
                          Register src2, Register srcA,
                          SBit s, Condition cond) {

  Instr16 instr16;
  bool it_effect;
  if (!predictable_code_size() && match(&instr16, &it_effect, op1, dst, src1, Operand(src2), s, cond)) {
    ASSERT(it_effect == true);
    emit16(instr16, s, cond);
  } else {
    ASSERT(s == LeaveCC);
    Instr32 instr32;
    if (match(&instr32, op2, dst, src1, src2, srcA)) {
      emit32(instr32, cond);
    } else UNIMPLEMENTED();
  }
}

void Assembler::thumb_mul_long(thumb32::muldiv::Opcode op,
                               Register dstL,
                               Register dstH,
                               Register src1,
                               Register src2,
                               SBit s,
                               Condition cond) {
  ASSERT(s == LeaveCC);
  Instr32 instr;
  if (match(&instr, op, dstL, dstH, src1, src2)) {
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::thumb_bit_field(thumb32::plain_imm::Opcode op,
                               Register dst,
                               Register src,
                               int lsb,
                               int width,
                               Condition cond) {
  Instr32 instr;
  if (match(&instr, op, dst, src, lsb, width)) {
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

// Compute the offset from MemOperand (could be in a form of immediate or
// shifted register).
void Assembler::fold_rm(Register r, const MemOperand& x, Condition cond) {
  if (!x.rm().is_valid()) {
    // Immediate
    mov(r, Operand(x.offset()), LeaveCC, cond);
  } else {
    // Shifted register
    mov(r, Operand(x.rm(), x.shift_op(), x.shift_imm()), LeaveCC, cond);
  }
}

// Emit load/store
// is_load argument determines if the entire operation can kill rt. In that case
// rt can be ip, and we can preload big immediates in it.
void Assembler::addrmod23(thumb16::mem::Opcode op16[], thumb32::mem::Opcode op, Register rt,
                          const MemOperand& x, Condition cond, bool is_load) {

  if (!predictable_code_size()) {
    using namespace thumb16::mem;
    Instr16 instr16;
    for (int i = 0; op16[i] != thumb16::mem::NONE; i++) {
      if (match(&instr16, op16[i], rt, x)) {
        emit16(instr16, cond);
        return;
      }
    }
  }

  using namespace thumb32::mem;
  Instr32 instr32;
  // Try single thumb32 instruction
  if (match(&instr32, op, rt, x)) {
    emit32(instr32, cond);
    if (op == thumb32::mem::LDR && rt.is(pc)) {
      flush_cond();
    }
    return;
  } else if (x.rm().is_valid()) {
    // It didn't match, and it's a register offset.
    // Thumb supports only the basic mode with LSL,0; other
    // have to be emulated
    Register rn = x.rn();
    Register rm = x.rm();
    ShiftOp shift_op = x.shift_op();
    int shift_imm = x.shift_imm();

    switch(x.am()) {
      case Offset:
        ASSERT(!rn.is(ip) && !(!is_load && rt.is(ip)));
        add(ip, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(ip), cond, is_load);
        break;
      case NegOffset:
        ASSERT(!rn.is(ip) && !(!is_load && rt.is(ip)));
        sub(ip, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(ip), cond, is_load);
        break;
      case PreIndex: // pre-index with writeback
        add(rn, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(rn), cond, is_load);
        break;
      case PostIndex: // post-index with writeback
      case PostIndexX:
        ASSERT(!rn.is(ip) && !(!is_load && rt.is(ip)));
        add(ip, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(ip), cond, is_load);
        mov(rn, ip, LeaveCC, cond);
        break;
      case NegPreIndex: // negative pre-index with writeback
        sub(rn, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(rn), cond, is_load);
        break;
      case NegPostIndex: // negative post-index with writeback
      case NegPostIndexX:
        ASSERT(!rn.is(ip) && !(!is_load && rt.is(ip)));
        sub(ip, rn, Operand(rm, shift_op, shift_imm), LeaveCC, cond);
        addrmod23(op16, op, rt, MemOperand(ip), cond, is_load);
        mov(rn, ip, LeaveCC, cond);
        break;
      default:
        UNIMPLEMENTED(); // wat?
    }
  } else {
    // It's a big immediate, just fold and repeat
    ASSERT(!x.rn().is(ip) && !(!is_load && rt.is(ip)));
    fold_rm(ip, x, cond);
    addrmod23(op16, op, rt, MemOperand(x.rn(), ip, x.am()), cond, is_load);
  }
}

// Emit load/store dual
void Assembler::addrmod3d(thumb32::mem_dual::Opcode op, Register r1, Register r2,
                          const MemOperand& x, Condition cond) {
  Instr32 instr;
  if (match(&instr, op, r1, r2, x)) {
    emit32(instr, cond);
    return;
  } else {
    ASSERT(!x.rm().is_valid());
    ASSERT(!x.rn().is(ip) && !r1.is(ip) && !r2.is(ip));
    Register rn = x.rn();
    fold_rm(ip, x, cond);
    switch(x.am()) {
      case Offset:
        add(ip, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        break;
      case NegOffset:
        sub(ip, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        break;
      case PreIndex: // pre-index with writeback
        add(rn, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        break;
      case PostIndex: // post-index with writeback
      case PostIndexX:
        add(ip, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        mov(rn, ip, LeaveCC, cond);
        break;
      case NegPreIndex: // negative pre-index with writeback
        sub(rn, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        break;
      case NegPostIndex: // negative post-index with writeback
      case NegPostIndexX:
        sub(ip, rn, Operand(ip), LeaveCC, cond);
        addrmod3d(op, r1, r2, MemOperand(ip), cond);
        mov(rn, ip, LeaveCC, cond);
        break;
      default:
        UNIMPLEMENTED();
    }
  }
}

// Emit load/store multiple
void Assembler::addrmod4(thumb16::mem_multi::Opcode op16_1,
                         thumb16::mem_multi::Opcode op16_2,
                         thumb32::mem_multi::Opcode op32,
                         BlockAddrMode am, Register base,
                         RegList regs, Condition cond) {

  if (!predictable_code_size()) {
    Instr16 instr16;
    if (match(&instr16, op16_1, am, base, regs) ||
        match(&instr16, op16_2, am, base, regs)) {
      emit16(instr16, cond);
      return;
    }
  }

  Instr32 instr32;
  if (NumberOfBitsSet(regs) == 1) {
    Register r = no_reg;
    for (int i = 0; i < kNumRegisters; i++) {
      if ((regs & (1 << i)) != 0) {
        r = Register::from_code(i);
        break;
      }
    }
    // Degenerate case that is not supported on thumb 32bit,
    // we'll just emit a single ldr/str.
    using namespace thumb32::mem_multi;
    using namespace thumb32::mem;
    MemOperand o(base, 4, AddrMode(am));
    if ((op32 == LDM && match(&instr32, LDR, r, o)) ||
        (op32 == STM && match(&instr32, STR, r, o))) {
      emit32(instr32, cond);
      return;
    }
  } else if (match(&instr32, op32, am, base, regs)) {
    emit32(instr32, cond);
    return;
  }
  UNIMPLEMENTED();
}

void Assembler::it(Condition cond, int16_t mask) {
  using namespace thumb16;
  ASSERT(is_uint4(mask));
  ASSERT(it_size_ == 0);
  emit16(misc::IT | ConditionToCode(cond) * B4 | B0 * mask, kSpecialCondition);
}

bool Assembler::match(Instr32 *instr, thumb32::mod_imm::Opcode op, Register rd, Register rn, const Operand& x, SBit s) {
  using namespace thumb32::mod_imm;
  if (op == NONE || x.rm().is_valid()) return false;

  uint32_t imm = x.immediate();
  int rot1 = 0;
  int rot3 = 0;
  int imm8 = (imm & 0xff);
  if (!is_uintn(imm, 8)) {
    // try shifts..
    int i = 31;
    while (i >= 8) {                         // looking for
      if ((imm & ~(0xff << (i - 7))) == 0 && // 8bit pattern anywhere in the in word
          (imm & (1 << i)) != 0) {           // with first bit == 1
        break;
      }
      i--;
    }
    // bad pattern, bail
    if (i < 8) return false;

    // encode
    imm8 = imm >> (i - 7);
    ASSERT((imm8 & ~0xff) == 0);
    ASSERT((imm8 & 0x80) != 0);

    int rot = 8 + (31 - i);
    ASSERT(rot <= 0x1f);

    // split rot into rot1:rot3:imm1
    rot1 = (rot >> 4) & 0x1;
    rot3 = (rot >> 1) & 0x7;
    int imm1 = rot & 0x1;
    // now imm1 must be shoved into the upper bit of imm8
    imm8 = (imm8 & 0x7f) | (imm1 << 7);
  }

  const Instr32 prototype = op | s | rot1 * H10 | rot3 * L12 | imm8 * L0;
  switch(op) {
    case TST: case TEQ:
      ASSERT(!rd.is_valid());
      ASSERT(s == SetCC); // s is already encoded in the op
      if (!rn.is_pcsp()) {
        *instr = prototype | rn.code() * H0;
        return true;
      }
      break;

    case CMP: case CMN:
      ASSERT(!rd.is_valid());
      ASSERT(s == SetCC); // s is already encoded in the op
      if (!rn.is(pc)) {
        *instr = prototype | rn.code() * H0;
        return true;
      }
      break;
    case AND: case BIC: case ORR: case ORN: case EOR: case ADC: case SBC: case RSB:
      if (!rd.is_pcsp() && !rn.is_pcsp()) {
        *instr = prototype | s | rn.code() * H0 | rd.code() * L8;
        return true;
      }
      break;
    case ADD: case SUB:
      // A8.8.4 ADD (immediate, Thumb)
      // A8.8.221 SUB (immediate, Thumb)
      // A8.8.9 ADD (SP plus immediate)
      // A8.8.225 SUB (SP minus immediate)
      if (!rd.is(pc) && !rn.is(pc) && (rn.is(sp) || !rd.is(sp))) {
        // If rn is sp, sp is allowed to be a destination, otherwise it's not. Weird.
        *instr = prototype | s | rn.code() * H0 | rd.code() * L8;
        return true;
      }
      break;
    case MOV: case MVN:
      ASSERT(!rn.is_valid());
      // A8.8.102 MOV (immediate)
      // A8.8.115 MVN (immediate)
      if (!rd.is_pcsp()) {
        *instr = prototype | s | rd.code() * L8;
        return true;
      }
      break;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32 *instr, thumb32::plain_imm::Opcode op, Register rd, Register rn, const Operand& x, SBit s) {
  using namespace thumb32::plain_imm;
  if (op == NONE || x.rm().is_valid() || s != LeaveCC) return false;
  uint32_t imm = x.immediate();

  if (!is_uint16(imm)) return false;
  // Split the immediate into the imm4:imm1:imm3:imm8 form
  const int imm4 = (imm & 0xffff) >> 12;
  const int imm1 = (imm & 0xfff)  >> 11;
  const int imm3 = (imm & 0x7ff)  >> 8;
  const int imm8 = (imm & 0xff);

  // Compose the common bits
  const Instr32 prototype = op | rd.code() * L8 | imm1 * H10 | imm3 * L12 | imm8 * L0;
  switch(op) {
    case ADDW: case SUBW: case ADR1: case ADR2:
      // A8.8.4 ADD (immediate, Thumb)
      // A8.8.221 SUB (immediate, Thumb)
      // A8.8.9 ADD (SP plus immediate)
      // A8.8.225 SUB (SP minus immediate)
      if (is_uintn(imm, 12) && !rd.is(pc) && (rn.is(sp) || !rd.is(sp))) {
        // If rn is sp, sp is allowed to be a destination, otherwise it's not.
        // Also this particular form allows to use pc as rn. It's ok to use ADDW & SUBW with
        // pc instead or ADR1 and ADR2 (rn will just be or'ed over the same value already set in the op).
        // imm = imm1:imm3:imm8
        *instr = prototype | rn.code() * H0;
        return true;
      }
      break;
    case MOVW: case MOVT:
      ASSERT(!rn.is_valid());
      // A8.8.102 MOV (immediate)
      if (!rd.is_pcsp() && is_uintn(imm, 16)) {
        // imm = imm4:imm1:imm3:imm8
        *instr = prototype | imm4 * H0;
        return true;
      }
      break;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::plain_imm::Opcode op, Register rd, const Operand& x, int satpos) {
  using namespace thumb32::plain_imm;
  if (op == NONE) return false;
  if (x.shift_op_ != ASR && x.shift_op_ != LSL) return false;

  Register rn = x.rm();
  if (rd.is_pcsp() || rn.is_pcsp()) return false;

  ASSERT((satpos >= 0) && (satpos <= 31));
  // Split lsb into the imm3:imm2 form
  const int imm3 = (x.shift_imm() & 0x1f) >> 2;
  const int imm2 = (x.shift_imm() & 0x3);

  const bool sh = x.shift_op_ == ASR;
  using namespace thumb32::plain_imm;
  switch(op) {
    case USAT: case SSAT:
      *instr = op | sh * H5 | rn.code() * H0 | imm3 * L12 | rd.code() * L8 | imm2 * L6 | satpos * L0;
      break;
    default:
      UNIMPLEMENTED();
  }
  return true;
}

bool Assembler::match(Instr32 *instr, thumb32::plain_imm::Opcode op, Register rd, Register rn, int lsb, int width) {
  using namespace thumb32::plain_imm;
  if (op == NONE || rd.is_pcsp() || rn.is_pcsp()) return false;

  ASSERT((lsb >= 0) && (lsb <= 31));
  ASSERT((width >= 1) && (width <= (32 - lsb)));

  // Split lsb into the imm3:imm2 form
  const int imm3 = (lsb & 0x1f) >> 2;
  const int imm2 = (lsb & 0x3);

  *instr = op | imm3 * L12 | rd.code() * L8 | imm2 * L6;
  switch(op) {
    case SBFX: case UBFX:
      *instr |= rn.code() * H0 | (width - 1) * L0;
      break;
    case BFI:
      *instr |= rn.code() * H0;
      // fall through..
    case BFC: {
      int msb = lsb + width - 1;
      *instr |=  msb;
      break;
    }
    default:
      UNIMPLEMENTED();
  }
  return true;
}


bool Assembler::match(Instr32* instr, thumb32::shifted_reg::Opcode op, Register rd, Register rn, const Operand& x, SBit s) {
  using namespace thumb32::shifted_reg;

  // Can't move immediates or put shift values in registers
  if (op == NONE || !x.rm().is_valid() || x.rs().is_valid() || x.rm().is(pc)) return false;

  Register rm = x.rm();
  const int shift_imm = x.shift_imm();

  // The shift value must be split into imm3:imm2 form.
  const int imm3 = shift_imm >> 2;
  const int imm2 = shift_imm & 3;

  // Put them in place
  const Instr32 prototype = op | s | (x.shift_op() >> kShiftOpBias) * L4 | imm3 * L12 | imm2 * L6 | rm.code() * L0;


  switch(op) {
    case MOV:
      ASSERT(!rn.is_valid());
      // pure MOV (shift_imm == 0, shift_op = LSL) has a special case allowing sp moves,
      // as long as there is no CC being set, and rd and rm are not sp at the same time.
      if (!rd.is(pc) && s == LeaveCC && x.is_reg() && !(rd.is(sp) && rm.is(sp))) {
        *instr = prototype | rd.code() * L8;
        return true;
      }
      // Generic move falls through...
    case MVN:
      ASSERT(!rn.is_valid());
      // Generic move
      if (!rd.is_pcsp() && !rm.is(sp)) {
        *instr = prototype | rd.code() * L8;
        return true;
      }
      break;

    case TST: case TEQ: case CMN: case CMP:
      ASSERT(s == SetCC);
      ASSERT(!rd.is_valid());
      if (!rm.is(sp) && !rn.is_pcsp()) {
        *instr = prototype | rn.code() * H0;
        return true;
      }
      break;
    case ADD: case SUB:
      if (rn.is(sp)) {
        if ((!rd.is(sp) || (x.shift_op() == LSL && shift_imm <= 3)) &&
            !rd.is(pc) && !rm.is(sp)) {
          *instr = prototype | rn.code() * H0 | rd.code() * L8;
          return true;
        } else {
          break;
        }
      }
      // else fall through..
    case AND: case EOR: case BIC: case ADC: case SBC:
    case RSB: case ORR:
      if (!rd.is_pcsp() && !rm.is(sp) && !rn.is_pcsp()) {
        *instr = prototype | rn.code() * H0 | rd.code() * L8;
        return true;
      }
      break;
    case PKH:
      if (!rd.is_pcsp() && !rm.is(sp) && !rn.is_pcsp() &&
          (x.shift_op() == LSL || x.shift_op() == ASR) && s == LeaveCC) {
        *instr = prototype | rn.code() * H0 | rd.code() * L8;
        return true;
      }
      break;
    default:
      UNIMPLEMENTED();
  }
  return false;
}


bool Assembler::match(Instr32* instr, ShiftOp shift_op, Register rd, Register rn, const Operand& x, SBit s) {
  if (shift_op != LSL && shift_op != LSR &&
      shift_op != ASR && shift_op != ROR) return false;
  if (!x.is_reg()) return false;

  Register rm = x.rm();
  if (rd.is_pcsp() || rn.is_pcsp() || rm.is_pcsp()) return false;

  *instr = thumb32::data_proc::SHIFT | (shift_op >> kShiftOpBias) * H5 | s | rn.code() * H0 | rd.code() * L8 | rm.code() * L0;
  return true;
}

bool Assembler::match(Instr32* instr, thumb32::data_proc::Opcode op, Register rd, Register rn, const Operand& x, SBit s) {
  using namespace thumb32;
  if (op == data_proc::NONE) return false;

  Register rm = x.rm();
  if (rd.is_pcsp() || rm.is_pcsp()) return false;

  Instr32 prototype = op | rd.code() * L8 | rm.code() * L0;
  switch(op) {
    case data_proc::LSL: case data_proc::LSR: case data_proc::ASR: case data_proc::ROR:
      if (!rn.is_pcsp() && x.shift_op() == LSL && x.shift_imm() == 0) {
        *instr = prototype | s | rn.code() * H0;
        return true;
      }
      break;
    case data_proc::UXTAB: case data_proc::UXTB: case data_proc::UXTB16:
      if (s == LeaveCC && x.shift_op() == ROR && (x.shift_imm() & 7) == 0) {
        prototype |= (x.shift_imm() >> 3) * L4;
        if (op == data_proc::UXTAB) {
          if (rn.is_pcsp()) break;
          prototype |= rn.code() * H0;
        } else {
          ASSERT(!rn.is_valid());
          if (rn.is_valid()) return false;
        }
        *instr = prototype;
        return true;
      }
      break;
    default:
      UNIMPLEMENTED();
  }

  return false;
}

bool Assembler::match(Instr32* instr, thumb32::mem::Opcode op, Register rt, const MemOperand& x) {
  using namespace thumb32::mem;
  if (op == NONE) return false;

  Register rn = x.rn();
  int am = x.am();

  Register rm = x.rm();
  Instr32 prototype = op | rn.code() * H0 | rt.code() * L12;

  if (rm.is_valid()) { // register offset
    // Thumb doesn't support complex addressing modes with a register offset -
    // no pre/post increments, etc. So if anything complex is required, we'll
    // have to fold the offset expression in a temporary first.
    if (am != Offset) return false;
    if (x.shift_op() != LSL || rm.is_pcsp()) return false;
    if (!is_uintn(x.shift_imm(), 2)) return false;

    ASSERT(am == Offset);
    prototype |= x.shift_imm() * L4 | rm.code() * L0;
  } else { // immediate offset
    int32_t offset = x.offset();
    if (offset < 0) {
      offset = -offset;
      am ^= U;
    }
    // stores can't ever have rn == pc, but loads can (of course).
    if (rn.is(pc) && (op == STR || op == STRH || op == STRB)) {
      ASSERT(false);
      return false;
    }
    if (am == Offset || (rn.is(pc) && am == NegOffset)) {
      // Simple offset supports a longer range - 12 bits.
      if (!is_uintn(offset, 12)) return false;
      if (rn.is(pc)) { // If rn is pc, sign is allowed
        prototype |= H7 * am_bit(am, U);
      } else {
        prototype |= H7;
      }
    } else {
      // If that's a prefetch, the only addressing mode that is supported is
      // NegOffset.
      if (op == PLD && am != NegOffset) return false;

      // Complex addressing mode
      ASSERT(am_bit(am, P) || am_bit(am, W));
      ASSERT(!am_bit(am, W) || !rt.is(rn));
      ASSERT(!rn.is(pc));
      if ((!am_bit(am, P) && !am_bit(am, W)) ||
          (am_bit(am, W) && rt.is(rn)) ||
          !is_uintn(offset, 8) || rn.is(pc)) return false;

      prototype |= L11 | am_bit(am, P) * L10 | am_bit(am, U) * L9 | am_bit(am, W) * L8;
    }
    prototype |= offset * L0;
  }
  switch(op) {
    case LDRH: case LDRSH: case LDRB: case LDRSB:
      ASSERT(!rt.is(pc));
      if (rt.is(sp)) return false;
      // else fall through
    case LDR:
      *instr = prototype;
      return true;
    case STRH: case STRB:
      if (rt.is(sp)) return false;
      // else fall through
    case STR:
      if (rt.is(pc)) return false;
      *instr = prototype;
      return true;
    case PLD:
      ASSERT(rt.is(pc));
      if (!rt.is(pc)) return false;
      *instr = prototype;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::mul::Opcode op, Register rd, Register rn, Register rm, Register ra) {
  using namespace thumb32::mul;
  if (op == NONE || rd.is_pcsp() || rn.is_pcsp() || rm.is_pcsp() || ra.is_pcsp()) return false;
  Instr32 prototype = op | rd.code() * L8 | rn.code() * H0 | rm.code() * L0;
  switch(op) {
    case MLA: case MLS:
      *instr = prototype | ra.code() * L12;
      return true;
    case MUL:
      *instr = prototype;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::muldiv::Opcode op, Register rd, Register rn, Register rm) {
  using namespace thumb32::muldiv;
  if (op == NONE || rd.is_pcsp() || rn.is_pcsp() || rm.is_pcsp()) return false;
  Instr32 prototype = op | (L15 | L14 | L13 | L12) | rn.code() * H0 | rd.code() * L8 | rm.code() * L0;
  switch(op) {
    case SDIV:
      *instr = prototype;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::muldiv::Opcode op, Register rdlo, Register rdhi, Register rn, Register rm) {
  using namespace thumb32::muldiv;
  if (op == NONE) return false;
  ASSERT(!rdlo.is(rdhi));
  if (rdlo.is_pcsp() || rdhi.is_pcsp() || rn.is_pcsp() || rm.is_pcsp() || rdlo.is(rdhi)) return false;
  Instr32 prototype = op | rn.code() * H0 | rdlo.code() * L12 | rdhi.code() * L8 | rm.code() * L0;
  switch(op) {
    case SMULL: case UMULL: case SMLAL: case UMLAL:
      *instr = prototype;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::mem_dual::Opcode op, Register rt1, Register rt2, const MemOperand& x) {
  using namespace thumb32::mem_dual;
  if (op == NONE) return false;
  ASSERT(x.rn().is_valid());
  if (rt1.is_pcsp() || rt2.is_pcsp() || x.rm().is_valid() ||
      !x.rn().is_valid() || (x.offset() & 3) != 0) return false;

  int32_t offset = x.offset();
  int am = x.am();

  if (offset < 0) {
    offset = -offset;
    am ^= U;
  }

  ASSERT((x.offset() & 3) == 0); // must be 4 byte aligned on thumb
  offset >>= 2;

  Register rn = x.rn();
  if (!is_uintn(offset, 8)) return false;
  switch(op) {
    case LDRD:
      ASSERT(am_bit(am, P) || am_bit(am, W));
      ASSERT(!rt1.is(rt2));
      if (rt1.is(rt2)) return false;
      if (am_bit(am, W)) {
        ASSERT(!rn.is(pc));
        if (rn.is(pc)) return false;
        else if (rn.is(rt1) || rn.is(rt2)) return false;
      }
      break;
    case STRD:
      ASSERT(am_bit(am, P) || am_bit(am, W));
      if (rn.is(pc)) return false;
      if (am_bit(am, W) && (rn.is(rt1) || rn.is(rt2))) return false;
      break;
    default:
      UNIMPLEMENTED();
  }
  *instr = op | am_bit(am, P) * H8 | am_bit(am, U) * H7 | am_bit(am, W) * H5 |
    rn.code() * H0 | rt1.code() * L12 | rt2.code() * L8 | offset * L0;
  return true;
}

// stm doesn't work with either pc or sp
// ldm doesn't work with sp, and if pc and lr are simultaniously used.
// these cases have to be emulated with multiple instructions.
bool Assembler::match(Instr32* instr, thumb32::mem_multi::Opcode op, BlockAddrMode am, Register rn, RegList rl) {
  using namespace thumb32::mem_multi;
  if (op == NONE) return false;
  // it's either inc after or dec before, other modes are unsupported, hopefully other are not used
  ASSERT((am & bm_x) == ia_x || (am & bm_x) == db_x);
  ASSERT(rl != 0);

  if ((rl & sp.bit()) != 0) return false;

  Instr32 prototype = op | rn.code() * H0 | rl * L0 | am_bit(am, W) * H5 | rl * L0;
  if ((am & bm_x) == ia_x) {
   prototype |= H7;
  } else if ((am & bm_x) == db_x) {
    prototype |= H8;
  } else UNREACHABLE();

  ASSERT(!rn.is(pc));
  ASSERT(NumberOfBitsSet(rl) >= 2);
  ASSERT((am & W) == 0 || (rl & rn.bit()) == 0);
  switch(op) {
    case STM:
      if ((rl & pc.bit()) != 0) return false;
      break;
    case LDM:
      if ((am & bm_x) == db_x) return false; // this is officially unsupported
      if ((rl & pc.bit()) != 0 && (rl & lr.bit()) != 0) return false;
      break;
    default:
      UNIMPLEMENTED();
  }

  *instr = prototype;
  return true;
}

bool Assembler::match(Instr32* instr, thumb32::misc::Opcode op, Register rd, Register rm) {
  using namespace thumb32::misc;
  if (op == NONE) return false;
  switch(op) {
    case CLZ:
      if (rd.is_pcsp() || rm.is_pcsp()) return false;
      *instr = op | rm.code() * H0 | rd.code() * L8 | rm.code() * L0;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr32* instr, thumb32::control::Opcode op, const Operand& x, Condition cond) {
  using namespace thumb32::control;
  if (op == NONE || x.rm().is_valid()) return false;
  ASSERT((x.immediate() & 1) == 0);
  const int32_t offset = x.immediate() >> 1;
  switch(op) {
    case COND_B: {
      if (cond == al) return false; // can't do always, try another encoding
      if (!is_intn(offset, 20)) return false; // does it fit?
      *instr = op | ConditionToCode(cond) * H6 | thumb32::CondBranchEncodeOffset(offset);
      break;
    }
    case AL_B:
      if (cond != al) return false;
    case BL: {
      if (!is_int24(offset)) return false; // does it fit?
      *instr = op | thumb32::BranchEncodeOffset(offset);
      break;
    }
    default:
      UNIMPLEMENTED();
  }
  ASSERT(thumb32::GetBranchOffset(*instr) == x.immediate());
  return true;
}

bool Assembler::match(Instr16* instr, thumb16::control::Opcode op, const Operand& x, Condition cond) {
  using namespace thumb16::control;
  if (op == NONE || x.rm().is_valid()) return false;
  ASSERT((x.immediate() & 1) == 0);
  const int32_t offset = x.immediate() >> 1;
  switch(op) {
    case COND_B: {
      if (cond == al) return false; // can't do always, try another encoding
      if (!is_intn(offset, 8)) return false; // does it fit?
      *instr = op | ConditionToCode(cond) * B8 | (offset & 0xff) * B0;
      return true;
    }
    case AL_B: {
      if (cond != al) return false;
      if (!is_intn(offset, 11)) return false; // does it fit?
      *instr = op | (offset & 0x7ff) * B0;
      return true;
    }
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr16* instr, bool* it_effect, thumb16::data_proc1::Opcode op,
                      Register rd, Register rn, const Operand& x, SBit s, Condition cond) {
  using namespace thumb16::data_proc1;
  if (op == NONE || (x.rm().is_valid() && x.rs().is_valid())) return false;

  Register rm = x.rm();
  switch(op) {
    case MOV:
      if (s == SetCC && cond != al) return false;
      ASSERT(!rn.is_valid());
      if (rm.is_valid() && is_uint3(rd.code()) && is_uint3(rm.code())) {
        ASSERT(is_uint5(x.shift_imm()));
        switch(x.shift_op()) {
          case LSL: case LSR: case ASR:
            break;
          default:
            return false;
        }
        *instr = op | (x.shift_op() >> kShiftOpBias) * B11 | x.shift_imm() * B6 | rm.code() * B3 | rd.code() * B0;
        *it_effect = true;
        return true;
      }
      break;
    case ADD: case SUB:
      if (s == SetCC && cond != al) return false;
      if (x.is_reg() && is_uint3(rd.code()) && is_uint3(rm.code()) && is_uint3(rn.code())) {
        *instr = op | rm.code() * B6 | rn.code() * B3 | rd.code() * B0;
        *it_effect = true;
        return true;
      }
      break;
    case ADD_IMM3: case SUB_IMM3:
      if (s == SetCC && cond != al) return false;
      if (!rm.is_valid() && is_uint3(x.immediate()) && is_uint3(rd.code()) && is_uint3(rn.code())) {
        *instr = op | x.immediate() * B6 | rn.code() * B3 | rd.code() * B0;
        *it_effect = true;
        return true;
      }
      break;
    case ADD_IMM8: case SUB_IMM8:
      if (s == SetCC && cond != al) return false;
      if (!rm.is_valid() && rd.is(rn) && is_uint3(rn.code()) && is_uint8(x.immediate())) {
        *instr = op | rn.code() * B8 | x.immediate() * B0;
        *it_effect = true;
        return true;
      }
      break;
    case MOV_IMM8:
      if (s == SetCC && cond != al) return false;
      ASSERT(!rn.is_valid());
      if (!rm.is_valid() && is_uint8(x.immediate()) && is_uint3(rd.code())) {
        *instr = op | rd.code() * B8 | x.immediate() * B0;
        *it_effect = true;
        return true;
      }
      break;
    case CMP_IMM8:
      ASSERT(s == SetCC);
      ASSERT(!rd.is_valid());
      if (!rm.is_valid() && is_uint8(x.immediate()) && is_uint3(rn.code())) {
        *instr = op | rn.code() * B8 | x.immediate() * B0;
        *it_effect = false;
        return true;
      }
      break;
    case ADD_PC_IMM8:
      if (s == LeaveCC && rn.is(pc) && !rm.is_valid() && is_uint3(rd.code())) {
        int imm = x.immediate();
        if ((imm & 3) != 0 || !is_uint10(imm)) return false;
        *instr = op | rd.code() * B8 | (imm >> 2) * B0;
        *it_effect = false;
        return true;
      }
      break;
    case ADD_SP_IMM8:
      if (s == LeaveCC && rn.is(sp) && !rm.is_valid() && is_uint3(rd.code())) {
        int imm = x.immediate();
        if ((imm & 3) != 0 || !is_uint10(imm)) return false;
        *instr = op | rd.code() * B8 | (imm >> 2) * B0;
        *it_effect = false;
        return true;
      }
      break;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr16* instr, bool* it_effect, thumb16::data_proc2::Opcode op,
                      Register rd, Register rn, const Operand& x, SBit s, Condition cond) {
  if (op == thumb16::data_proc2::NONE || (x.rm().is_valid() && x.rs().is_valid())) return false;
  if (!x.rm().is_valid()) {
    // One instruction in the family supports implied immediate
    if (op == thumb16::data_proc2::RSB && x.immediate() == 0 &&
        is_uint3(rd.code()) && is_uint3(rn.code())) {
      if (s == SetCC && cond != al) return false;
      *instr = op | rn.code() * B3 | rd.code() * B0;
      *it_effect = true;
      return true;
    }
    return false;
  } else if (x.shift_op() == LSL && x.shift_imm() == 0) {
    Register rm = x.rm();
    switch(op) {
      using namespace thumb16;
      using namespace thumb16::data_proc2;
      // There are commutative, so we can try permuting rn and rm
      case AND: case EOR: case ORR: case ADC:
        {
          if (s == SetCC && cond != al) break;
          Register r = no_reg;
          if (rd.is(rn)) {
            r = rm;
          } else if (rd.is(rm)) {
            r = rn;
          } else break;
          if (is_uint3(rd.code()) && is_uint3(r.code())) {
            *instr = op | r.code() * B3 | rd.code() * B0;
            *it_effect = true;
            return true;
          }
        }
        break;
      case data_proc2::LSL: case data_proc2::LSR: case data_proc2::ASR:
      case data_proc2::ROR: case BIC: case SBC:
        if (s == SetCC && cond != al) break;
        if (rd.is(rn) && is_uint3(rd.code()) && is_uint3(rm.code())) {
          *instr = op | rm.code() * B3 | rd.code() * B0;
          *it_effect = true;
          return true;
        }
        break;
      case TST: case CMP: case CMN:
        ASSERT(s == SetCC);
        ASSERT(!rd.is_valid());
        if (is_uint3(rn.code()) && is_uint3(rm.code())) {
          *instr = op | rm.code() * B3 | rn.code() * B0;
          *it_effect = false;
          return true;
        }
        break;
      case MUL:
        {
          if (s == SetCC && cond != al) break;
          Register r = no_reg;
          if (rd.is(rn)) {
            r = rm;
          } else if (rd.is(rm)) {
            r = rn;
          } else break;
          if (is_uint3(rd.code()) && is_uint3(r.code())) {
            *instr = op | r.code() * B3 | rd.code() * B0;
            *it_effect = true;
            return true;
          }
          break;
        }
      case MVN:
        ASSERT(!rn.is_valid());
        if (s == SetCC && cond != al) break;
        if (is_uint3(rd.code()) && is_uint3(rm.code())) {
          *instr = op | rm.code() * B3 | rd.code() * B0;
          *it_effect = true;
          return true;
        }
        break;
      case RSB: break;
      default:
        UNIMPLEMENTED();
    }
  }
  return false;
}

bool Assembler::match(Instr16* instr, bool* it_effect, ShiftOp shift_op, Register rd, Register rn, const Operand& x, SBit s, Condition cond) {
  using namespace thumb16;
  data_proc2::Opcode op = data_proc2::NONE;
  switch(shift_op) {
    case LSL: op = data_proc2::LSL; break;
    case LSR: op = data_proc2::LSR; break;
    case ASR: op = data_proc2::ASR; break;
    case ROR: op = data_proc2::ROR; break;
    default: return false;
  }
  return match(instr, it_effect, op, rd, rn, x, s, cond);
}

bool Assembler::match(Instr16* instr, bool* it_effect, thumb16::spec_data::Opcode op,
                      Register rd, Register rn, const Operand& x, SBit s) {
  using namespace thumb16::spec_data;
  if (op == NONE) return false;
  Register rm = x.rm();
  if (!rm.is_valid() || x.rs().is_valid() || x.shift_op() != LSL || x.shift_imm() != 0) return false;
  Instr16 prototype = op | rm.code() * B3;
  switch(op) {
    case CMP:
      ASSERT(s == SetCC);
      if (rn.is(pc) || rm.is(pc)) return false;
      if (rn.code() < 8 && rm.code() < 8) return false;
      *instr = prototype | (rn.code() >> 3) * B7 | (rn.code() & 0x7) * B0;
      *it_effect = false;
      return true;
    case ADD:
      if (s == SetCC) return false;
      if (!rd.is(rn) || (rn.is(pc) && rm.is(pc))) return false;
      // fall through..
    case MOV:
      if (s == SetCC) return false;
      *instr = prototype | (rd.code() >> 3) * B7 | (rd.code() & 0x7) * B0;
      *it_effect = false;
      return true;
    case BX: case BLX:
      *instr = prototype;
      *it_effect = false;
      return true;
    default:
      UNIMPLEMENTED();
  }
  return false;
}

bool Assembler::match(Instr16* instr, thumb16::mem::Opcode op, Register rt, const MemOperand& x) {
  using namespace thumb16::mem;
  if (op == NONE || x.am() != Offset || !is_uint3(rt.code())) return false;

  Register rn = x.rn();
  Register rm = x.rm();
  if (rn.is(sp) || rn.is(pc)) {
    if (!rm.is_valid() &&
        ((rn.is(sp) && (op == STR_SP_IMM || op == LDR_SP_IMM)) ||
         (rn.is(pc) && op == LDR_PC_IMM))) {
      int imm = x.offset();
      if (imm & 3) return false;
      imm >>= 2;
      if (is_uint8(imm)) {
        *instr = op | rt.code() * B8 | imm * B0;
        return true;
      }
    }
    return false;
  } else {
    if (!is_uint3(rn.code())) return false;
    if (rm.is_valid()) {
      // Register offset
      // No support for shifts
      if (x.shift_op() != LSL || x.shift_imm() != 0) return false;
      if (is_uint3(rm.code())) {
        switch(op) {
          case STR: case STRH: case STRB: case LDRSB:
          case LDR: case LDRH: case LDRB: case LDRSH:
            *instr = op | rm.code() * B6 | rn.code() * B3 | rt.code() * B0;
            return true;
          default:
            return false;
        }
      }
      return false;
    } else {
      // Immediate offset
      int imm = x.offset();
      Instr16 prototype = op | rn.code() * B3 | rt.code() * B0;
      switch(op) {
        case STR_IMM: case LDR_IMM:
          if (imm & 3) return false;
          imm >>= 2;
          break;
        case STRH_IMM: case LDRH_IMM:
          if (imm & 1) return false;
          imm >>= 1;
          break;
        case STRB_IMM: case LDRB_IMM:
          break;
        default:
          return false;
      }
      if (is_uint5(imm)) {
        *instr = prototype | imm * B6;
        return true;
      }
      return false;
    }
  }
  UNREACHABLE();
}

bool Assembler::match(Instr16* instr, thumb16::mem_multi::Opcode op, BlockAddrMode am, Register rn, RegList rl) {
  using namespace thumb16::mem_multi;
  if (op == NONE) return false;
  ASSERT(NumberOfBitsSet(rl) > 0);
  if (rn.is(sp)) {
    const int imm8 = rl & 0xff;
    Instr16 prototype = op | imm8 * B0;
    switch(op) {
      case PUSH:
        if (am == db_w && (rl & 0xbf00) == 0) { // only lr and lower 8 registers
          *instr = prototype | ((rl & lr.bit()) != 0) * B8;
          return true;
        }
        break;
      case POP:
        if (am == ia_w && (rl & 0x7f00) == 0) { // only pc and lower 8 registers
          *instr = prototype | ((rl & pc.bit()) != 0) * B8;
          return true;
        }
        break;
      default:
        break;
    }
  } else {
    // lower registers only in the set and the base in also in [r0-r7]
    if ((rl & 0xff00) != 0 || !is_uint3(rn.code())) return false;
    // wback = (rl[rn] == 0) and inc-after only
    if (!am_bit(am, W) != (rl & rn.bit()) || ((am & bm_x) != ia_x)) return false;
    if (op == LDM || op == STM) {
      *instr = op | rn.code() * B8 | rl * B0;
      return true;
    }
  }
  return false;
}

int Assembler::target_at(int pos)  {
  using namespace thumb32;
  Instr16 maybe_it = instr16_at(pos);
  if (thumb16::IsIT(maybe_it)) {
    pos += kInstrSize;
#ifdef DEBUG
    Instr32 next_instr = instr32_at(pos);
    ASSERT(IsUnconditionalBranch(next_instr) || IsBranchAndLink(next_instr));
#endif
  }
  Instr32 instr = instr32_at(pos);
  if ((instr & T32PrefixMask) == 0) {
    // Emitted label constant, not part of a branch.
    return instr - (Code::kHeaderSize - kHeapObjectTag);
  }
  return pos + kPcLoadDelta + GetBranchOffset(instr);
}


void Assembler::target_at_put(int pos, int target_pos) {
  using namespace thumb32;
  Instr16 maybe_it = instr16_at(pos);
  if (thumb16::IsIT(maybe_it)) {
    pos += kInstrSize;
#ifdef DEBUG
    Instr32 next_instr = instr32_at(pos);
    ASSERT(IsUnconditionalBranch(next_instr) || IsBranchAndLink(next_instr));
#endif
  }
  Instr32 instr = instr32_at(pos);
  if ((instr & T32PrefixMask) == 0) {
    // Emitted label constant, not part of a branch.
    // Make label relative to Code* of generated Code object.
    instr32_at_put(pos, target_pos + (Code::kHeaderSize - kHeapObjectTag));
    return;
  }
  // unconditional b, conditional b, bl, blx
  const int imm = target_pos - (pos + kPcLoadDelta);
  instr32_at_put(pos, SetBranchOffset(instr, imm));
}


void Assembler::print(Label* L) {
  using namespace thumb32;
  using namespace thumb32::control;
  if (L->is_unused()) {
    PrintF("unused label\n");
  } else if (L->is_bound()) {
    PrintF("bound label to %d\n", L->pos());
  } else if (L->is_linked()) {
    Label l = *L;
    PrintF("unbound label");
    while (l.is_linked()) {
      PrintF("@ %d ", l.pos());
      Instr32 instr = instr32_at(l.pos());
      if ((instr & T32PrefixMask) == 0) {
        PrintF("value\n");
      } else {
        const char* b = "";
        const char* c = ConditionToString(GetBranchCondition(instr));
        Opcode op = static_cast<Opcode>(instr & BRANCH_MASK);
        switch(op) {
          case COND_B:
            b = "b";
            break;
          case AL_B:
            b = "b";
            break;
          case BLX:
            b = "blx";
            break;
          case BL:
            b = "bl";
            break;
          default:
            UNREACHABLE();
        }
        PrintF("%s%s\n", b, c);
      }
      next(&l);
    }
  } else {
    PrintF("label in inconsistent state (pos = %d)\n", L->pos_);
  }
}

static bool use_movw_movt(const Operand& x, const Assembler* assembler) {
  if (Assembler::use_immediate_embedded_pointer_loads(assembler)) {
    return true;
  }
  if (x.must_output_reloc_info(assembler)) {
    return false;
  }
  return CpuFeatures::IsSupported(ARMv7);
}


void Assembler::move_32_bit_immediate(Condition cond,
                                      Register rd,
                                      SBit s,
                                      const Operand& x) {
  ASSERT(s == LeaveCC);

  int imm = x.immediate();
  if (RelocInfo::IsCodeTarget(x.rmode()) || RelocInfo::IsRuntimeEntry(x.rmode())) {
    // If we're emitting a pointer to the code, set the lower bit to 1
    // to indicate that the target is thumb.
    imm = CPU::EncodePc(imm);
  }

  if (rd.code() != pc.code() && use_movw_movt(x, this)) {
    if (x.must_output_reloc_info(this)) {
      // End IT block if necessary, we need movw and movt to stay together
      // for patching.
      flush_cond();
      cond32(cond, false); // Just start the block if necessary
      RecordRelocInfo(x.rmode(), imm, DONT_USE_CONSTANT_POOL);
      // Make sure the movw/movt doesn't get separated.
      BlockConstPoolFor(4); // 4 here because we count in small instructions
    }
    movw_(rd, imm & 0xffff, cond);
    movt(rd, static_cast<uint32_t>(imm) >> 16, cond);
    return;
  }

  cond32(cond, false);
  RecordRelocInfo(x.rmode(), imm, USE_CONSTANT_POOL);
  ldr(rd, MemOperand(pc, 0), cond);
}


// Branch instructions.
void Assembler::b(int branch_offset, Condition c, bool offset_is_final) {
  bool done = false;
  if (offset_is_final && !predictable_code_size()) {
    using namespace thumb16;
    Instr16 instr;
    if (match(&instr, control::COND_B, Operand(branch_offset), c) ||
        match(&instr, control::AL_B, Operand(branch_offset), c)) {
      flush_cond(); // end IT block before, if there's any
      emit16(instr, kSpecialCondition);
      done = true;
    }
  }
  if (!done) {
    using namespace thumb32;
    Instr32 instr;
    if (c == al) {
      if (match(&instr, control::AL_B, Operand(branch_offset), c)) {
        flush_cond(); // end IT block before, if there's any
        emit32(instr, kSpecialCondition);
      } else UNIMPLEMENTED();
    } else {
      if (match(&instr, control::COND_B, Operand(branch_offset), c)) {
        flush_cond(); // end IT block before, if there's any
        emit32(instr, kSpecialCondition);
      } else {
        // The range is probably to short. Let's try an IT block with
        // unconditioncal jump. That gives us 16x more range.
        b_long(branch_offset, c, offset_is_final);
      }
    }
  }

  if (c == al) {
    // Dead code is a good location to emit the constant pool.
    CheckConstPool(false, false);
  }
}


void Assembler::b_long(int branch_offset, Condition c, bool offset_is_final) {
  if (c == al || offset_is_final) {
    b(branch_offset, c, offset_is_final);
  } else {
    using namespace thumb32;
    Instr32 instr;
    ASSERT(c != al);
    if (match(&instr, control::AL_B, Operand(branch_offset - Assembler::kInstrSize), al)) {
      BlockConstPoolFor(3);
      flush_cond(); // end previous IT block before, if there's any
      cond(c, true); // emit an IT instruction with cond
      emit32(instr, kSpecialCondition);
      flush_cond(); // must be last in IT block
    } else UNIMPLEMENTED();
  }
}

void Assembler::bl(int branch_offset, Condition c) {
  positions_recorder()->WriteRecordedPositions();
  using namespace thumb32;
  Instr32 instr;
  if (c == al) {
    if (match(&instr, control::BL, Operand(branch_offset), al)) {
      flush_cond();
      emit32(instr, al);
    } else UNIMPLEMENTED();
  } else {
    if (match(&instr, control::BL, Operand(branch_offset - Assembler::kInstrSize), al)) {
      BlockConstPoolFor(3);
      flush_cond();
      // Conditional bl becomes and it-bl pair.
      // The patching machinery is aware of this pair, in fact it's
      // the same as for b_long.
      cond(c, true);
      emit32(instr, kSpecialCondition);
      flush_cond();
    } else UNIMPLEMENTED();
  }
}


void Assembler::blx(int branch_offset) {  // v5 and above
  // XXX: In thumb, blx with immediate switches back to arm mode,
  // and we don't want to support that at this time.
  // Since we are in thumb-only mode, we'll just translate that to "bl"
  bl(branch_offset, al);
}


void Assembler::blx(Register target, Condition cond) {  // v5 and above
  positions_recorder()->WriteRecordedPositions();
  using namespace thumb16::spec_data;
  Instr16 instr;
  bool ignore;

  if (match(&instr, &ignore, BLX, no_reg, no_reg, Operand(target), LeaveCC)) {
    emit16(instr, cond);
    flush_cond();
  } else UNIMPLEMENTED();
}


void Assembler::bx(Register target, Condition cond) {  // v5 and above, plus v4t
  positions_recorder()->WriteRecordedPositions();
  using namespace thumb16::spec_data;
  Instr16 instr;
  bool ignore;

  if (match(&instr, &ignore, BX, no_reg, no_reg, Operand(target), LeaveCC)) {
    emit16(instr, cond);
    flush_cond();
  } else UNIMPLEMENTED();
}

// choose between two options of addrmod1 operations
void Assembler::choose_imm(ThumbAddrMod1Options o1, ThumbAddrMod1Options o2,
    Register dst, Register src1, const Operand& src2_1, const Operand& src2_2,
    SBit s, Condition cond) {
  // If we must emit relocation, we can't trasnform immediates,
  // so only the primary option works.
  if (src2_1.must_output_reloc_info(this)) {
    addrmod1(o1, dst, src1, src2_1, s, cond);
  } else if (predictable_code_size()) {
    // In predictable mode we still allow to choose the second option,
    // if the first one is impossible to emit. The second options is
    // usually a dual (like and/bic, add/sub) of the primary operation
    // with the transformed argument.
    if (!addrmod1(o1, dst, src1, src2_1, s, cond, true)) {
      addrmod1(o2, dst, src1, src2_2, s, cond, false);
    }
  } else {
    // If we are not constrained by size choose the shorter form.
    // Here we consider both primary and secondary options in all forms and
    // choose the smallest in size.
    Sizer _o1(this), _o2(this);
    bool r1 = _o1.addrmod1(o1, dst, src1, src2_1, s, cond, true);
    bool r2 = _o2.addrmod1(o2, dst, src1, src2_2, s, cond, true);

    if ((r1 && !r2) || (r1 && r2 && _o1.SizeOfCode() <= _o2.SizeOfCode())) {
      addrmod1(o1, dst, src1, src2_1, s, cond);
    } else {
      ASSERT(r2);
      addrmod1(o2, dst, src1, src2_2, s, cond);
    }
  }
}


void Assembler::and_(Register dst, Register src1, const Operand& src2,
                     SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::AND,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::AND,
      false /* not move */
    };
    addrmod1(o, dst, src1, src2, s, cond);
  } else {
    ThumbAddrMod1Options o_and_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::AND,
      mod_imm::AND, plain_imm::NONE, shifted_reg::AND,
      false /* not move */
    };
    ThumbAddrMod1Options o_bic_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::BIC,
      mod_imm::BIC, plain_imm::NONE, shifted_reg::BIC,
      false /* not move */
    };
    choose_imm(o_and_imm, o_bic_imm, dst, src1, src2, Operand(~src2.immediate()), s, cond);
  }
  Kill(dst);
}


void Assembler::eor(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::EOR,
    mod_imm::EOR, plain_imm::NONE, shifted_reg::EOR,
    false /* not move */
  };
  Kill(dst);
  addrmod1(o, dst, src1, src2, s, cond);
}


void Assembler::sub(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::NONE,
      { data_proc1::SUB, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::SUB,
      false /* not move */
    };
    addrmod1(o, dst, src1, src2, s, cond);
  } else {
    ThumbAddrMod1Options o_add_imm = {
      spec_data::ADD,
      { data_proc1::ADD, data_proc1::ADD_IMM8, data_proc1::ADD_IMM3,
        data_proc1::ADD_SP_IMM8, data_proc1::ADD_PC_IMM8, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::ADD, plain_imm::ADDW, shifted_reg::ADD,
      false /* not move */
    };
    ThumbAddrMod1Options o_sub_imm = {
      spec_data::NONE,
      { data_proc1::SUB, data_proc1::SUB_IMM8, data_proc1::SUB_IMM3, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::SUB, plain_imm::SUBW, shifted_reg::SUB,
      false /* not move */
    };
    // immediate
    choose_imm(o_sub_imm, o_add_imm, dst, src1, src2, Operand(-src2.immediate(), src2.rmode()), s, cond);
  }
  Kill(dst);
}


void Assembler::rsb(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::RSB,
    mod_imm::RSB, plain_imm::NONE, shifted_reg::RSB,
    false /* not move */
  };
  Kill(dst);
  addrmod1(o, dst, src1, src2, s, cond);
}

void Assembler::add(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::ADD,
      { data_proc1::ADD, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::ADD,
      false /* not move */
    };
    addrmod1(o, dst, src1, src2, s, cond);
  } else {
    ThumbAddrMod1Options o_add_imm = {
      spec_data::ADD,
      { data_proc1::ADD, data_proc1::ADD_IMM8, data_proc1::ADD_IMM3,
        data_proc1::ADD_SP_IMM8, data_proc1::ADD_PC_IMM8, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::ADD, plain_imm::ADDW, shifted_reg::ADD,
      false /* not move */
    };
    ThumbAddrMod1Options o_sub_imm = {
      spec_data::NONE,
      { data_proc1::SUB, data_proc1::SUB_IMM8, data_proc1::SUB_IMM3, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::SUB, plain_imm::SUBW, shifted_reg::SUB,
      false /* not move */
    };
    // immediate
    choose_imm(o_add_imm, o_sub_imm, dst, src1, src2, Operand(-src2.immediate(), src2.rmode()), s, cond);
  }
  Kill(dst);
}


void Assembler::adc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::ADC,
    mod_imm::ADC, plain_imm::NONE, shifted_reg::ADC,
    false /* not move */
  };
  Kill(dst);
  addrmod1(o, dst, src1, src2, s, cond);
}


void Assembler::sbc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::SBC,
    mod_imm::SBC, plain_imm::NONE, shifted_reg::SBC,
    false /* not move */
  };
  addrmod1(o, dst, src1, src2, s, cond);
  Kill(dst);
}


void Assembler::rsc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  // this is supported only on classic arm, we'll have to emulate it
  // but it seems that V8 is not emitting it.
  UNIMPLEMENTED();
}


void Assembler::tst(Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::TST,
    mod_imm::TST, plain_imm::NONE, shifted_reg::TST,
    false /* not move */
  };
  addrmod1(o, no_reg, src1, src2, SetCC, cond);
}


void Assembler::teq(Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::NONE,
    mod_imm::TEQ, plain_imm::NONE, shifted_reg::TEQ,
    false /* not move */
  };
  addrmod1(o, no_reg, src1, src2, SetCC, cond);
}

void Assembler::cmp(Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::CMP,
      { data_proc1::CMP_IMM8, data_proc1::NONE },
      data_proc2::CMP,
      mod_imm::CMP, plain_imm::NONE, shifted_reg::CMP,
      false /* not move */
    };
    addrmod1(o, no_reg, src1, src2, SetCC, cond);
  } else {
    ThumbAddrMod1Options o_cmp_imm = {
      spec_data::NONE,
      { data_proc1::CMP_IMM8, data_proc1::NONE },
      data_proc2::CMP,
      mod_imm::CMP, plain_imm::NONE, shifted_reg::CMP,
      false // not move
    };
    ThumbAddrMod1Options o_cmn_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::CMN,
      mod_imm::CMN, plain_imm::NONE, shifted_reg::CMN,
      false // not move
    };
    // immediate
    choose_imm(o_cmp_imm, o_cmn_imm, no_reg, src1, src2, Operand(-src2.immediate(), src2.rmode()), SetCC, cond);
  }
}

// Special instruction (mostly side-effect free) that is used to encode offsets,
// in the register and immediate fields. Shouldn't be used in any other context,
// because the actual comparison will fail - the imm1 and imm3 below actually
// encode rotations in the normal settings. So, if you want a compare - use
// the regular cmp().
// See also: JumpPatchSize::EmitPatchInfo().
void Assembler::cmp_raw_immediate(Register src, int raw_immediate, Condition cond) {
  using namespace thumb32;
  // A8.8.37 CMP (immediate) encoding T2
  Instr32 instr = SetCmpImmediateRawImmediate(SetCmpImmediateRegister(mod_imm::CMP, src), raw_immediate);
  ASSERT(IsCmpImmediate(instr));
  ASSERT(GetCmpImmediateRegister(instr).is(src));
  ASSERT(GetCmpImmediateRawImmediate(instr) == raw_immediate);
  emit32(instr);
}


void Assembler::cmn(Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::CMN,
      mod_imm::CMN, plain_imm::NONE, shifted_reg::CMN,
      false /* not move */
    };
    addrmod1(o, no_reg, src1, src2, SetCC, cond);
  } else {
    ThumbAddrMod1Options o_cmp_imm = {
      spec_data::NONE,
      { data_proc1::CMP_IMM8, data_proc1::NONE },
      data_proc2::CMP,
      mod_imm::CMP, plain_imm::NONE, shifted_reg::CMP,
      false // not move
    };
    ThumbAddrMod1Options o_cmn_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::CMN,
      mod_imm::CMN, plain_imm::NONE, shifted_reg::CMN,
      false // not move
    };
    // immediate
    choose_imm(o_cmn_imm, o_cmp_imm, no_reg, src1, src2, Operand(-src2.immediate(), src2.rmode()), SetCC, cond);
  }
}


void Assembler::orr(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  ThumbAddrMod1Options o = {
    spec_data::NONE,
    { data_proc1::NONE },
    data_proc2::ORR,
    mod_imm::ORR, plain_imm::NONE, shifted_reg::ORR,
    false /* not move */
  };
  Kill(dst);
  addrmod1(o, dst, src1, src2, s, cond);
}

void Assembler::bic(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src2.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::BIC,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::BIC,
      false /* not move */
    };
    addrmod1(o, dst, src1, src2, s, cond);
  } else {
    ThumbAddrMod1Options o_and_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::AND,
      mod_imm::AND, plain_imm::NONE, shifted_reg::AND,
      false /* not move */
    };
    ThumbAddrMod1Options o_bic_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::BIC,
      mod_imm::BIC, plain_imm::NONE, shifted_reg::BIC,
      false /* not move */
    };
    choose_imm(o_bic_imm, o_and_imm, dst, src1, src2, Operand(~src2.immediate()), s, cond);
  }
  Kill(dst);
}


void Assembler::mvn(Register dst, const Operand& src, SBit s, Condition cond) {
  using namespace thumb32;
  using namespace thumb16;
  if (src.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::MVN,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::MVN,
      false /* not move */
    };
    addrmod1(o, dst, no_reg, src, s, cond);
  } else {
    ThumbAddrMod1Options o_mov_imm = {
      spec_data::MOV,
      { data_proc1::MOV, data_proc1::MOV_IMM8, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::MOV, plain_imm::MOVW, shifted_reg::MOV,
      true /* is move */
    };
    ThumbAddrMod1Options o_mvn_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::MVN,
      mod_imm::MVN, plain_imm::NONE, shifted_reg::MVN,
      false /* not move */
    };
    choose_imm(o_mvn_imm, o_mov_imm, dst, no_reg, src, Operand(~src.immediate()), s, cond);
  }
  Kill(dst);
}


void Assembler::mov(Register dst, const Operand& src, SBit s, Condition cond) {
  if (dst.is(pc)) {
    positions_recorder()->WriteRecordedPositions();
  }
  // Don't allow nop instructions in the form mov rn, rn to be generated using
  // the mov instruction. They must be generated using nop(int/NopMarkerTypes)
  // or MarkCode(int/NopMarkerTypes) pseudo instructions.
  ASSERT(!(src.is_reg() && src.rm().is(dst) && s == LeaveCC && cond == al));

  using namespace thumb32;
  using namespace thumb16;
  if (src.rm().is_valid()) {
    ThumbAddrMod1Options o = {
      spec_data::MOV,
      { data_proc1::MOV, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::NONE, plain_imm::NONE, shifted_reg::MOV,
      true /* is move */
    };
    addrmod1(o, dst, no_reg, src, s, cond);
  } else {
    ThumbAddrMod1Options o_mov_imm = {
      spec_data::MOV,
      { data_proc1::MOV, data_proc1::MOV_IMM8, data_proc1::NONE },
      data_proc2::NONE,
      mod_imm::MOV, plain_imm::MOVW, shifted_reg::MOV,
      true /* is move */
    };
    ThumbAddrMod1Options o_mvn_imm = {
      spec_data::NONE,
      { data_proc1::NONE },
      data_proc2::MVN,
      mod_imm::MVN, plain_imm::NONE, shifted_reg::MVN,
      false /* not move */
    };
    choose_imm(o_mov_imm, o_mvn_imm, dst, no_reg, src, Operand(~src.immediate()), s, cond);
  }
  Kill(dst);
}


void Assembler::movw(Register reg, uint32_t immediate, Condition cond) {
  // movw is always supported with Thumb2
  movw_(reg, immediate, cond);
}

// Pure movw
void Assembler::movw_(Register reg, uint32_t immediate, Condition cond) {
  // movw is already supported with Thumb2
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, plain_imm::MOVW, reg, no_reg, Operand(immediate), LeaveCC)) {
    Kill(reg);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::movt(Register reg, uint32_t immediate, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, plain_imm::MOVT, reg, no_reg, Operand(immediate), LeaveCC)) {
    Kill(reg);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}


void Assembler::mla(Register dst, Register src1, Register src2, Register srcA,
                    SBit s, Condition cond) {
  using namespace thumb16;
  using namespace thumb32;
  Kill(dst);
  thumb_mul(data_proc2::NONE, mul::MLA, dst, src1, src2, srcA, s, cond);
}

void Assembler::mls(Register dst, Register src1, Register src2, Register srcA,
                    Condition cond) {
  using namespace thumb16;
  using namespace thumb32;
  Kill(dst);
  thumb_mul(data_proc2::NONE, mul::MLS, dst, src1, src2, srcA, LeaveCC, cond);
}

void Assembler::mul(Register dst, Register src1, Register src2,
                    SBit s, Condition cond) {

  // Try 16-bit instruction first..
  using namespace thumb16;
  using namespace thumb32;
  thumb_mul(data_proc2::MUL, mul::MUL, dst, src1, src2, no_reg, s, cond);
  Kill(dst);
}

void Assembler::sdiv(Register dst, Register src1, Register src2,
                     Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, muldiv::SDIV, dst, src1, src2)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}


void Assembler::smlal(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace thumb32;
  Kill(dstL);
  Kill(dstH);
  thumb_mul_long(muldiv::SMLAL, dstL, dstH, src1, src2, s, cond);
}


void Assembler::smull(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace thumb32;
  Kill(dstL);
  Kill(dstH);
  thumb_mul_long(muldiv::SMULL, dstL, dstH, src1, src2, s, cond);
}


void Assembler::umlal(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace thumb32;
  Kill(dstL);
  Kill(dstH);
  thumb_mul_long(muldiv::UMLAL, dstL, dstH, src1, src2, s, cond);
}


void Assembler::umull(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace thumb32;
  Kill(dstL);
  Kill(dstH);
  thumb_mul_long(muldiv::UMULL, dstL, dstH, src1, src2, s, cond);
}


// Miscellaneous arithmetic instructions.
void Assembler::clz(Register dst, Register src, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, misc::CLZ, dst, src)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}


// Saturating instructions.

// Unsigned saturate.
void Assembler::usat(Register dst,
                     int satpos,
                     const Operand& src,
                     Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, plain_imm::USAT, dst, src, satpos)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}


// Bitfield manipulation instructions.
// Unsigned bit field extract.
// Extracts #width adjacent bits from position #lsb in a register, and
// writes them to the low bits of a destination register.
//   ubfx dst, src, #lsb, #width
void Assembler::ubfx(Register dst,
                     Register src,
                     int lsb,
                     int width,
                     Condition cond) {
  using namespace thumb32;
  Kill(dst);
  thumb_bit_field(plain_imm::UBFX, dst, src, lsb, width, cond);
}


// Signed bit field extract.
// Extracts #width adjacent bits from position #lsb in a register, and
// writes them to the low bits of a destination register. The extracted
// value is sign extended to fill the destination register.
//   sbfx dst, src, #lsb, #width
void Assembler::sbfx(Register dst,
                     Register src,
                     int lsb,
                     int width,
                     Condition cond) {
  using namespace thumb32;
  Kill(dst);
  thumb_bit_field(plain_imm::SBFX, dst, src, lsb, width, cond);
}


// Bit field clear.
// Sets #width adjacent bits at position #lsb in the destination register
// to zero, preserving the value of the other bits.
//   bfc dst, #lsb, #width
void Assembler::bfc(Register dst, int lsb, int width, Condition cond) {
  using namespace thumb32;
  Kill(dst);
  thumb_bit_field(plain_imm::BFC, dst, no_reg, lsb, width, cond);
}


// Bit field insert.
// Inserts #width adjacent bits from the low bits of the source register
// into position #lsb of the destination register.
//   bfi dst, src, #lsb, #width
void Assembler::bfi(Register dst,
                    Register src,
                    int lsb,
                    int width,
                    Condition cond) {
  using namespace thumb32;
  Kill(dst);
  thumb_bit_field(plain_imm::BFI, dst, src, lsb, width, cond);
}


void Assembler::pkhbt(Register dst, Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  ASSERT((src2.shift_imm_ >= 0) && (src2.shift_imm_ <= 31));
  ASSERT(src2.shift_op() == LSL);
  Instr32 instr;
  if (match(&instr, shifted_reg::PKH, dst, src1, src2, LeaveCC)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::pkhtb(Register dst, Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  ASSERT((src2.shift_imm_ >= 1) && (src2.shift_imm_ <= 32));
  ASSERT(src2.shift_op() == ASR);
  // Special encoding case. Shift by 32 bits is encoded as a shift by 0 bits.
  int asr = (src2.shift_imm_ == 32) ? 0 : src2.shift_imm_;
  Instr32 instr;
  if (match(&instr, shifted_reg::PKH, dst, src1, Operand(src2.rm(), src2.shift_op(), asr), LeaveCC)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::uxtb(Register dst, const Operand& src, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, data_proc::UXTB, dst, no_reg, src, LeaveCC)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::uxtab(Register dst, Register src1, const Operand& src2, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, data_proc::UXTAB, dst, src1, src2, LeaveCC)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}

void Assembler::uxtb16(Register dst, const Operand& src, Condition cond) {
  using namespace thumb32;
  Instr32 instr;
  if (match(&instr, data_proc::UXTB16, dst, no_reg, src, LeaveCC)) {
    Kill(dst);
    emit32(instr, cond);
  } else UNIMPLEMENTED();
}


// Status register access instructions.
void Assembler::mrs(Register dst, SRegister s, Condition cond) {
  using namespace thumb32;
  ASSERT(!dst.is_pcsp());
  Kill(dst);
  emit32(control::MRS | dst.code() * L8, cond);
}


void Assembler::msr(SRegisterFieldMask fields, const Operand& src,
                    Condition cond) {
  ASSERT(fields >= B16 && fields < B20);  // at least one field set
  using namespace thumb32;
  if (!src.rm().is_valid()) {
    // Immediate (there is no msr version with immediate in thumb2, use ip as a
    // scratch register).
    mov(ip, src);
    msr(fields, Operand(ip), cond);
  } else {
    // Register
    emit32(control::MSR | src.rm().code() * H0 | (fields >> SRegisterBias) * L8, cond);
  }
}



namespace thumb32 {

static MemOperand FixAddrMode(MemOperand x) {
  // Fixup the addressing mode:
  // Classic ARM implies a writeback if a post increment is specified.
  // And the existing AddrMode encoding assumes that behavior, that's why
  // all AddrMode::.*PostIndex have W == 0.
  //
  // Thumb doesn't assume the writeback, however it outlaws the case when
  // P == 0 && W == 0.
  // So, we'll just set the W bit any time P == 0.
  if (am_bit(x.am(), P) == 0) { // postindex implies writeback
    x.set_am(AddrMode(x.am() | W));
  }
  return x;
}

}

// Load/Store instructions.
void Assembler::ldr(Register dst, const MemOperand& src, Condition cond) {
  if (dst.is(pc)) {
    positions_recorder()->WriteRecordedPositions();
  }
  Kill(dst);

  bool done = false;
  MemOperand x = thumb32::FixAddrMode(src);
  if (am_bit(x.am(), W)) Kill(x.rn());
  if (!predictable_code_size()) {
    // Check if it's a pop, there is a 16-bit instruction for that
    if (x.rn().is(sp) && !x.rm().is_valid()) {
      BlockAddrMode bm = BlockAddrMode(x.am());
      int32_t offset = x.offset();
      if (bm == ia_w && offset == 4) {
        using namespace thumb16::mem_multi;
        Instr16 instr;
        if (match(&instr, POP, ia_w, sp, dst.bit())) {
          emit16(instr, cond);
          done = true;
        }
      }
    }
  }

  if (!done) {
    using namespace thumb16::mem;
    Opcode op16[] = { LDR, LDR_IMM, LDR_SP_IMM, NONE };
    using namespace thumb32;
    addrmod23(op16, mem::LDR, dst, x, cond, true /*is_load*/);
  }

  if (dst.is(pc)) {
    flush_cond();
  }
}

void Assembler::str(Register src, const MemOperand& dst, Condition cond) {
  MemOperand x = thumb32::FixAddrMode(dst);
  if (am_bit(x.am(), W)) Kill(x.rn());
  if (!predictable_code_size()) {
    // Check if it's a push, there is a 16-bit instruction for that
    if (x.rn().is(sp) && !x.rm().is_valid()) {
      BlockAddrMode bm = BlockAddrMode(x.am());
      int32_t offset = x.offset();
      if (bm == db_w && offset == 4) {
        using namespace thumb16::mem_multi;
        Instr16 instr;
        if (match(&instr, PUSH, db_w, sp, src.bit())) {
          emit16(instr, cond);
          return;
        }
      }
    }
  }
  using namespace thumb16::mem;
  Opcode op16[] = { STR, STR_IMM, STR_SP_IMM, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::STR, src, x, cond, false /*is_load*/);
}


void Assembler::ldrb(Register dst, const MemOperand& src, Condition cond) {
  Kill(dst);
  using namespace thumb16::mem;
  Opcode op16[] = { LDRB, LDRB_IMM, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::LDRB, dst, FixAddrMode(src), cond, true /*is_load*/);
}


void Assembler::strb(Register src, const MemOperand& dst, Condition cond) {
  using namespace thumb16::mem;
  Opcode op16[] = { STRB, STRB_IMM, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::STRB, src, FixAddrMode(dst), cond, false /*is_load*/);
}


void Assembler::ldrh(Register dst, const MemOperand& src, Condition cond) {
  Kill(dst);
  using namespace thumb16::mem;
  Opcode op16[] = { LDRH, LDRH_IMM, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::LDRH, dst, FixAddrMode(src), cond, true /*is_load*/);
}


void Assembler::strh(Register src, const MemOperand& dst, Condition cond) {
  using namespace thumb16::mem;
  Opcode op16[] = { STRH, STRH_IMM, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::STRH, src, FixAddrMode(dst), cond, false /*is_load*/);
}


void Assembler::ldrsb(Register dst, const MemOperand& src, Condition cond) {
  Kill(dst);
  using namespace thumb16::mem;
  Opcode op16[] = { LDRSB, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::LDRSB, dst, FixAddrMode(src), cond, true /*is_load*/);
}


void Assembler::ldrsh(Register dst, const MemOperand& src, Condition cond) {
  Kill(dst);
  using namespace thumb16::mem;
  Opcode op16[] = { LDRSH, NONE };
  using namespace thumb32;
  addrmod23(op16, mem::LDRSH, dst, FixAddrMode(src), cond, true /*is_load*/);
}


void Assembler::ldrd(Register dst1, Register dst2,
                     const MemOperand& src, Condition cond) {
  using namespace thumb32;
  ASSERT(src.rm().is(no_reg));
  Kill(dst1);
  Kill(dst2);
  addrmod3d(mem_dual::LDRD, dst1, dst2, FixAddrMode(src), cond);
}


void Assembler::strd(Register src1, Register src2,
                     const MemOperand& dst, Condition cond) {
  using namespace thumb32;
  ASSERT(dst.rm().is(no_reg));
  addrmod3d(mem_dual::STRD, src1, src2, FixAddrMode(dst), cond);
}


void Assembler::pld(const MemOperand& address) {
  using namespace thumb16::mem;
  Opcode op16[] = { NONE };
  using namespace thumb32;
  addrmod23(op16, mem::PLD, pc, FixAddrMode(address), al, false /*is_load*/);
}

// Load/Store multiple instructions.
void Assembler::ldm(BlockAddrMode am,
                    Register base,
                    RegList dst,
                    Condition cond) {
  Kill(dst);
  if (am_bit(am, W)) Kill(base);
  addrmod4(thumb16::mem_multi::LDM, thumb16::mem_multi::POP,
           thumb32::mem_multi::LDM, am, base, dst, cond);
  // Emit the constant pool after a function return implemented by ldm ..{..pc}.
  if (cond == al && (dst & pc.bit()) != 0) {
    // Must be the last instruction in an IT block
    flush_cond();
    // There is a slight chance that the ldm instruction was actually a call,
    // in which case it would be wrong to return into the constant pool; we
    // recognize this case by checking if the emission of the pool was blocked
    // at the pc of the ldm instruction by a mov lr, pc instruction; if this is
    // the case, we emit a jump over the pool.
    CheckConstPool(true, no_const_pool_before_ == pc_offset() - kInstrSize);
  }
}


void Assembler::stm(BlockAddrMode am,
                    Register base,
                    RegList src,
                    Condition cond) {
  if (am_bit(am, W)) Kill(base);
  // Emulate IB & DA modes
  if ((am & bm_x) == ib_x || (am & bm_x) == da_x) {
    Register tmp = base;
    if (!am_bit(am, W)) {
      ASSERT(!base.is(ip));
      ASSERT((src & ip.bit()) == 0);
      tmp = ip;
    }
    am = static_cast<BlockAddrMode>(am ^ P);
    add(tmp, base, Operand(4), LeaveCC, cond);
    stm(am, tmp, src, cond);
    if (am_bit(am, W)) {
      sub(base, tmp, Operand(4), LeaveCC, cond);
    }
  } else  {
    addrmod4(thumb16::mem_multi::STM, thumb16::mem_multi::PUSH,
             thumb32::mem_multi::STM, am, base, src, cond);
  }
}


void Assembler::bkpt(uint32_t imm) {  // v5 and above
  using namespace thumb16;
  ASSERT(is_uint8(imm));
  emit16(misc::BKPT | imm, al);
}


void Assembler::svc(uint32_t imm, Condition cond) {
  using namespace thumb16;
  ASSERT(is_uint8(imm));
  emit16(misc::SVC | imm, cond);
}


// Pseudo instructions.
void Assembler::nop(int type) {
  // ARMv6{K/T2} and v7 have an actual NOP instruction but it serializes
  // some of the CPU's pipeline and has to issue. Older ARM chips simply used
  // MOV Rx, Rx as NOP and it performs better even in newer CPUs.
  // We therefore use MOV Rx, Rx, even on newer CPUs, and use Rx to encode
  // a type.
  ASSERT(0 <= type && type <= 14);  // mov pc, pc isn't a nop.
  Instr16 instr;
  bool ignore;
  Register r = Register::from_code(type);
  // This move is magical - it's a 16bit instruction,
  // it doesn't set condition flags (A8.8.103 encoding T1),
  // and has a full range of registers.
  bool m = match(&instr, &ignore, thumb16::spec_data::MOV, r, no_reg, Operand(r), LeaveCC);
  ASSERT(m == true);
  if (m) {
    // Check encode/decode consistency
    ASSERT(thumb16::IsNop(instr, type));
    // nop is always unconditional, so we close the possible IT block
    flush_cond();
    // kSpecialCondition tells the emitter to skip the IT block logic
    emit16(instr, kSpecialCondition);
  }
}

} }  // namespace v8::internal

#endif
