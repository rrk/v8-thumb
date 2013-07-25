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
// Copyright 2012 the V8 project authors. All rights reserved.

#include "v8.h"

#if defined(V8_TARGET_ARCH_ARM)

#include "arm/assembler-arm-inl.h"
#include "serialize.h"

namespace v8 {
namespace internal {

int Assembler::target_at(int pos)  {
  using namespace arm;
  Instr instr = instr_at(pos);
  if ((instr & ~kImm24Mask) == 0) {
    // Emitted label constant, not part of a branch.
    return instr - (Code::kHeaderSize - kHeapObjectTag);
  }
  ASSERT((instr & 7*B25) == 5*B25);  // b, bl, or blx imm24
  int imm26 = ((instr & kImm24Mask) << 8) >> 6;
  if ((Instruction::ConditionField(instr) == kSpecialCondition) &&
      ((instr & B24) != 0)) {
    // blx uses bit 24 to encode bit 2 of imm26
    imm26 += 2;
  }
  return pos + kPcLoadDelta + imm26;
}


void Assembler::target_at_put(int pos, int target_pos) {
  using namespace arm;
  Instr instr = instr_at(pos);
  if ((instr & ~kImm24Mask) == 0) {
    ASSERT(target_pos == kEndOfChain || target_pos >= 0);
    // Emitted label constant, not part of a branch.
    // Make label relative to Code* of generated Code object.
    instr_at_put(pos, target_pos + (Code::kHeaderSize - kHeapObjectTag));
    return;
  }
  int imm26 = target_pos - (pos + kPcLoadDelta);
  ASSERT((instr & 7*B25) == 5*B25);  // b, bl, or blx imm24
  if (Instruction::ConditionField(instr) == kSpecialCondition) {
    // blx uses bit 24 to encode bit 2 of imm26
    ASSERT((imm26 & 1) == 0);
    instr = (instr & ~(B24 | kImm24Mask)) | ((imm26 & 2) >> 1)*B24;
  } else {
    ASSERT((imm26 & 3) == 0);
    instr &= ~kImm24Mask;
  }
  int imm24 = imm26 >> 2;
  ASSERT(is_int24(imm24));
  instr_at_put(pos, instr | (imm24 & kImm24Mask));
}


void Assembler::print(Label* L) {
  using namespace arm;
  if (L->is_unused()) {
    PrintF("unused label\n");
  } else if (L->is_bound()) {
    PrintF("bound label to %d\n", L->pos());
  } else if (L->is_linked()) {
    Label l = *L;
    PrintF("unbound label");
    while (l.is_linked()) {
      PrintF("@ %d ", l.pos());
      Instr instr = instr_at(l.pos());
      if ((instr & ~kImm24Mask) == 0) {
        PrintF("value\n");
      } else {
        const char* b;
        const char* c = "";
        ASSERT((instr & 7*B25) == 5*B25);  // b, bl, or blx
        Condition cond = Instruction::ConditionField(instr);
        if (cond == kSpecialCondition) {
          b = "blx";
        } else {
          if ((instr & B24) != 0)
            b = "bl";
          else
            b = "b";
          c = ConditionToString(cond);
        }
        PrintF("%s%s\n", b, c);
      }
      next(&l);
    }
  } else {
    PrintF("label in inconsistent state (pos = %d)\n", L->pos_);
  }
}

// Low-level code emission routines depending on the addressing mode.
// If this returns true then you have to use the rotate_imm and immed_8
// that it returns, because it may have already changed the instruction
// to match them!
static bool fits_shifter(uint32_t imm32,
                         uint32_t* rotate_imm,
                         uint32_t* immed_8,
                         Instr* instr) {
  using namespace arm;
  // imm32 must be unsigned.
  for (int rot = 0; rot < 16; rot++) {
    uint32_t imm8 = (imm32 << 2*rot) | (imm32 >> (32 - 2*rot));
    if ((imm8 <= 0xff)) {
      *rotate_imm = rot;
      *immed_8 = imm8;
      return true;
    }
  }
  // If the opcode is one with a complementary version and the complementary
  // immediate fits, change the opcode.
  if (instr != NULL) {
    if ((*instr & kMovMvnMask) == kMovMvnPattern) {
      if (fits_shifter(~imm32, rotate_imm, immed_8, NULL)) {
        *instr ^= kMovMvnFlip;
        return true;
      } else if ((*instr & kMovLeaveCCMask) == kMovLeaveCCPattern) {
        if (CpuFeatures::IsSupported(ARMv7)) {
          if (imm32 < 0x10000) {
            *instr ^= kMovwLeaveCCFlip;
            *instr |= EncodeMovwImmediate(imm32);
            *rotate_imm = *immed_8 = 0;  // Not used for movw.
            return true;
          }
        }
      }
    } else if ((*instr & kCmpCmnMask) == kCmpCmnPattern) {
      if (fits_shifter(-static_cast<int>(imm32), rotate_imm, immed_8, NULL)) {
        *instr ^= kCmpCmnFlip;
        return true;
      }
    } else {
      Instr alu_insn = (*instr & kALUMask);
      if (alu_insn == ADD ||
          alu_insn == SUB) {
        if (fits_shifter(-static_cast<int>(imm32), rotate_imm, immed_8, NULL)) {
          *instr ^= kAddSubFlip;
          return true;
        }
      } else if (alu_insn == AND ||
                 alu_insn == BIC) {
        if (fits_shifter(~imm32, rotate_imm, immed_8, NULL)) {
          *instr ^= kAndBicFlip;
          return true;
        }
      }
    }
  }
  return false;
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
  if (rd.code() != pc.code() && use_movw_movt(x, this)) {
    if (x.must_output_reloc_info(this)) {
      RecordRelocInfo(x.rmode_, x.imm32_, DONT_USE_CONSTANT_POOL);
      // Make sure the movw/movt doesn't get separated.
      BlockConstPoolFor(2);
    }
    movw_(rd, x.imm32_ & 0xffff, cond);
    movt(rd, static_cast<uint32_t>(x.imm32_) >> 16, cond);
    return;
  }

  RecordRelocInfo(x.rmode_, x.imm32_, USE_CONSTANT_POOL);
  ldr(rd, MemOperand(pc, 0), cond);
}


void Assembler::addrmod1(Instr instr,
                         Register rn,
                         Register rd,
                         const Operand& x) {
  using namespace arm;
  CheckBuffer();
  ASSERT((instr & ~(kCondMask | kOpCodeMask | S)) == 0);
  if (!x.rm_.is_valid()) {
    // Immediate.
    uint32_t rotate_imm;
    uint32_t immed_8;
    if (x.must_output_reloc_info(this) ||
        !fits_shifter(x.imm32_, &rotate_imm, &immed_8, &instr)) {
      // The immediate operand cannot be encoded as a shifter operand, so load
      // it first to register ip and change the original instruction to use ip.
      // However, if the original instruction is a 'mov rd, x' (not setting the
      // condition code), then replace it with a 'ldr rd, [pc]'.
      CHECK(!rn.is(ip));  // rn should never be ip, or will be trashed
      Condition cond = Instruction::ConditionField(instr);
      if ((instr & ~kCondMask) == 13*B21) {  // mov, S not set
        move_32_bit_immediate(cond, rd, LeaveCC, x);
      } else {
        if ((instr & kMovMvnMask) == kMovMvnPattern) {
          // Moves need to use a constant pool entry.
          RecordRelocInfo(x.rmode_, x.imm32_, USE_CONSTANT_POOL);
          ldr(ip, MemOperand(pc, 0), cond);
        } else if (x.must_output_reloc_info(this)) {
          // Otherwise, use most efficient form of fetching from constant pool.
          move_32_bit_immediate(cond, ip, LeaveCC, x);
        } else {
          // If this is not a mov or mvn instruction we may still be able to
          // avoid a constant pool entry by using mvn or movw.
          mov(ip, x, LeaveCC, cond);
        }
        addrmod1(instr, rn, rd, Operand(ip));
      }
      return;
    }
    instr |= I | rotate_imm*B8 | immed_8;
  } else if (!x.rs_.is_valid()) {
    // Immediate shift.
    instr |= x.shift_imm_*B7 | x.shift_op_ | x.rm_.code();
  } else {
    // Register shift.
    ASSERT(!rn.is(pc) && !rd.is(pc) && !x.rm_.is(pc) && !x.rs_.is(pc));
    instr |= x.rs_.code()*B8 | x.shift_op_ | B4 | x.rm_.code();
  }
  emit(instr | rn.code()*B16 | rd.code()*B12);
  if (rn.is(pc) || x.rm_.is(pc)) {
    // Block constant pool emission for one instruction after reading pc.
    BlockConstPoolFor(1);
  }
}


void Assembler::addrmod2(Instr instr, Register rd, const MemOperand& x) {
  using namespace arm;
  ASSERT((instr & ~(kCondMask | B | L)) == B26);
  int am = x.am_;
  if (!x.rm_.is_valid()) {
    // Immediate offset.
    int offset_12 = x.offset_;
    if (offset_12 < 0) {
      offset_12 = -offset_12;
      am ^= U;
    }
    if (!is_uint12(offset_12)) {
      // Immediate offset cannot be encoded, load it first to register ip
      // rn (and rd in a load) should never be ip, or will be trashed.
      ASSERT(!x.rn_.is(ip) && ((instr & L) == L || !rd.is(ip)));
      mov(ip, Operand(x.offset_), LeaveCC, Instruction::ConditionField(instr));
      addrmod2(instr, rd, MemOperand(x.rn_, ip, x.am_));
      return;
    }
    ASSERT(offset_12 >= 0);  // no masking needed
    instr |= offset_12;
  } else {
    // Register offset (shift_imm_ and shift_op_ are 0) or scaled
    // register offset the constructors make sure than both shift_imm_
    // and shift_op_ are initialized.
    ASSERT(!x.rm_.is(pc));
    instr |= B25 | x.shift_imm_*B7 | x.shift_op_ | x.rm_.code();
  }
  ASSERT((am & (P|W)) == P || !x.rn_.is(pc));  // no pc base with writeback
  emit(instr | am | x.rn_.code()*B16 | rd.code()*B12);
}


void Assembler::addrmod3(Instr instr, Register rd, const MemOperand& x) {
  using namespace arm;
  ASSERT((instr & ~(kCondMask | L | S6 | H)) == (B4 | B7));
  ASSERT(x.rn_.is_valid());
  int am = x.am_;
  if (!x.rm_.is_valid()) {
    // Immediate offset.
    int offset_8 = x.offset_;
    if (offset_8 < 0) {
      offset_8 = -offset_8;
      am ^= U;
    }
    if (!is_uint8(offset_8)) {
      // Immediate offset cannot be encoded, load it first to register ip
      // rn (and rd in a load) should never be ip, or will be trashed.
      ASSERT(!x.rn_.is(ip) && ((instr & L) == L || !rd.is(ip)));
      mov(ip, Operand(x.offset_), LeaveCC, Instruction::ConditionField(instr));
      addrmod3(instr, rd, MemOperand(x.rn_, ip, x.am_));
      return;
    }
    ASSERT(offset_8 >= 0);  // no masking needed
    instr |= B | (offset_8 >> 4)*B8 | (offset_8 & 0xf);
  } else if (x.shift_imm_ != 0) {
    // Scaled register offset not supported, load index first
    // rn (and rd in a load) should never be ip, or will be trashed.
    ASSERT(!x.rn_.is(ip) && ((instr & L) == L || !rd.is(ip)));
    mov(ip, Operand(x.rm_, x.shift_op_, x.shift_imm_), LeaveCC,
        Instruction::ConditionField(instr));
    addrmod3(instr, rd, MemOperand(x.rn_, ip, x.am_));
    return;
  } else {
    // Register offset.
    ASSERT((am & (P|W)) == P || !x.rm_.is(pc));  // no pc index with writeback
    instr |= x.rm_.code();
  }
  ASSERT((am & (P|W)) == P || !x.rn_.is(pc));  // no pc base with writeback
  emit(instr | am | x.rn_.code()*B16 | rd.code()*B12);
}


void Assembler::addrmod4(Instr instr, Register rn, RegList rl) {
  using namespace arm;
  ASSERT((instr & ~(kCondMask | P | U | W | L)) == B27);
  ASSERT(rl != 0);
  ASSERT(!rn.is(pc));
  emit(instr | rn.code()*B16 | rl);
}

// Branch instructions.
void Assembler::b(int branch_offset, Condition cond, bool offset_is_final) {
  using namespace arm;
  ASSERT((branch_offset & 3) == 0);
  int imm24 = branch_offset >> 2;
  ASSERT(is_int24(imm24));
  emit(cond | B27 | B25 | (imm24 & kImm24Mask));

  if (cond == al) {
    // Dead code is a good location to emit the constant pool.
    CheckConstPool(false, false);
  }
}


void Assembler::bl(int branch_offset, Condition cond) {
  positions_recorder()->WriteRecordedPositions();
  using namespace arm;
  ASSERT((branch_offset & 3) == 0);
  int imm24 = branch_offset >> 2;
  ASSERT(is_int24(imm24));
  emit(cond | B27 | B25 | B24 | (imm24 & kImm24Mask));
}


void Assembler::blx(int branch_offset) {  // v5 and above
  using namespace arm;
  positions_recorder()->WriteRecordedPositions();
  ASSERT((branch_offset & 1) == 0);
  int h = ((branch_offset & 2) >> 1)*B24;
  int imm24 = branch_offset >> 2;
  ASSERT(is_int24(imm24));
  emit(kSpecialCondition | B27 | B25 | h | (imm24 & kImm24Mask));
}


void Assembler::blx(Register target, Condition cond) {  // v5 and above
  positions_recorder()->WriteRecordedPositions();
  using namespace arm;
  ASSERT(!target.is(pc));
  emit(cond | B24 | B21 | 15*B16 | 15*B12 | 15*B8 | BLX | target.code());
}


void Assembler::bx(Register target, Condition cond) {  // v5 and above, plus v4t
  positions_recorder()->WriteRecordedPositions();
  using namespace arm;
  ASSERT(!target.is(pc));  // use of pc is actually allowed, but discouraged
  emit(cond | B24 | B21 | 15*B16 | 15*B12 | 15*B8 | BX | target.code());
}


void Assembler::and_(Register dst, Register src1, const Operand& src2,
                     SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | AND | s, src1, dst, src2);
}


void Assembler::eor(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | EOR | s, src1, dst, src2);
}


void Assembler::sub(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | SUB | s, src1, dst, src2);
}


void Assembler::rsb(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | RSB | s, src1, dst, src2);
}


void Assembler::add(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | ADD | s, src1, dst, src2);
}


void Assembler::adc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | ADC | s, src1, dst, src2);
}


void Assembler::sbc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | SBC | s, src1, dst, src2);
}


void Assembler::rsc(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | RSC | s, src1, dst, src2);
}


void Assembler::tst(Register src1, const Operand& src2, Condition cond) {
  using namespace arm;
  addrmod1(cond | TST | S, src1, r0, src2);
}


void Assembler::teq(Register src1, const Operand& src2, Condition cond) {
  using namespace arm;
  addrmod1(cond | TEQ | S, src1, r0, src2);
}


void Assembler::cmp(Register src1, const Operand& src2, Condition cond) {
  using namespace arm;
  addrmod1(cond | CMP | S, src1, r0, src2);
}


void Assembler::cmp_raw_immediate(Register src, int raw_immediate, Condition cond) {
  using namespace arm;
  ASSERT(is_uint12(raw_immediate));
  emit(cond | I | CMP | S | src.code() << 16 | raw_immediate);
}


void Assembler::cmn(Register src1, const Operand& src2, Condition cond) {
  using namespace arm;
  addrmod1(cond | CMN | S, src1, r0, src2);
}


void Assembler::orr(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | ORR | s, src1, dst, src2);
}

void Assembler::bic(Register dst, Register src1, const Operand& src2,
                    SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | BIC | s, src1, dst, src2);
}


void Assembler::mvn(Register dst, const Operand& src, SBit s, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod1(cond | MVN | s, r0, dst, src);
}


void Assembler::mov(Register dst, const Operand& src, SBit s, Condition cond) {
  if (dst.is(pc)) {
    positions_recorder()->WriteRecordedPositions();
  }
  // Don't allow nop instructions in the form mov rn, rn to be generated using
  // the mov instruction. They must be generated using nop(int/NopMarkerTypes)
  // or MarkCode(int/NopMarkerTypes) pseudo instructions.
  ASSERT(!(src.is_reg() && src.rm().is(dst) && s == LeaveCC && cond == al));
  using namespace arm;
  Kill(dst);
  addrmod1(cond | MOV | s, r0, dst, src);
}


void Assembler::movw(Register reg, uint32_t immediate, Condition cond) {
  ASSERT(immediate < 0x10000);
  using namespace arm;
  // May use movw if supported, but on unsupported platforms will try to use
  // equivalent rotated immed_8 value and other tricks before falling back to a
  // constant pool load.
  mov(reg, Operand(immediate), LeaveCC, cond);
}


// Pure movw
void Assembler::movw_(Register reg, uint32_t immediate, Condition cond) {
  ASSERT(immediate < 0x10000);
  using namespace arm;
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  Kill(reg);
  emit(cond | 0x30*B20 | reg.code()*B12 | EncodeMovwImmediate(immediate));
}

void Assembler::movt(Register reg, uint32_t immediate, Condition cond) {
  ASSERT(immediate < 0x10000);
  using namespace arm;
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  Kill(reg);
  emit(cond | 0x34*B20 | reg.code()*B12 | EncodeMovwImmediate(immediate));
}




void Assembler::mla(Register dst, Register src1, Register src2, Register srcA,
                    SBit s, Condition cond) {
  using namespace arm;
  ASSERT(!dst.is(pc) && !src1.is(pc) && !src2.is(pc) && !srcA.is(pc));
  Kill(dst);
  emit(cond | A | s | dst.code()*B16 | srcA.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());

}


void Assembler::mls(Register dst, Register src1, Register src2, Register srcA,
                    Condition cond) {
  using namespace arm;
  ASSERT(!dst.is(pc) && !src1.is(pc) && !src2.is(pc) && !srcA.is(pc));
  Kill(dst);
  emit(cond | B22 | B21 | dst.code()*B16 | srcA.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());
}

void Assembler::mul(Register dst, Register src1, Register src2,
                    SBit s, Condition cond) {
  using namespace arm;
  ASSERT(!dst.is(pc) && !src1.is(pc) && !src2.is(pc));
  Kill(dst);
  // dst goes in bits 16-19 for this instruction!
  emit(cond | s | dst.code()*B16 | src2.code()*B8 | B7 | B4 | src1.code());
}

void Assembler::sdiv(Register dst, Register src1, Register src2,
                     Condition cond) {
  using namespace arm;
  ASSERT(!dst.is(pc) && !src1.is(pc) && !src2.is(pc));
  Kill(dst);
  emit(cond | B26 | B25| B24 | B20 | dst.code()*B16 | 0xf * B12 |
       src2.code()*B8 | B4 | src1.code());
}


void Assembler::smlal(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace arm;
  ASSERT(!dstL.is(pc) && !dstH.is(pc) && !src1.is(pc) && !src2.is(pc));
  ASSERT(!dstL.is(dstH));
  Kill(dstL);
  Kill(dstH);
  emit(cond | B23 | B22 | A | s | dstH.code()*B16 | dstL.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());
}


void Assembler::smull(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace arm;
  ASSERT(!dstL.is(pc) && !dstH.is(pc) && !src1.is(pc) && !src2.is(pc));
  ASSERT(!dstL.is(dstH));
  Kill(dstL);
  Kill(dstH);
  emit(cond | B23 | B22 | s | dstH.code()*B16 | dstL.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());
}


void Assembler::umlal(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace arm;
  ASSERT(!dstL.is(pc) && !dstH.is(pc) && !src1.is(pc) && !src2.is(pc));
  ASSERT(!dstL.is(dstH));
  Kill(dstL);
  Kill(dstH);
  emit(cond | B23 | A | s | dstH.code()*B16 | dstL.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());
}


void Assembler::umull(Register dstL,
                      Register dstH,
                      Register src1,
                      Register src2,
                      SBit s,
                      Condition cond) {
  using namespace arm;
  ASSERT(!dstL.is(pc) && !dstH.is(pc) && !src1.is(pc) && !src2.is(pc));
  ASSERT(!dstL.is(dstH));
  Kill(dstL);
  Kill(dstH);
  emit(cond | B23 | s | dstH.code()*B16 | dstL.code()*B12 |
       src2.code()*B8 | B7 | B4 | src1.code());
}


// Miscellaneous arithmetic instructions.
void Assembler::clz(Register dst, Register src, Condition cond) {
  ASSERT(cond == al);
  using namespace arm;
  // v5 and above.
  ASSERT(!dst.is(pc) && !src.is(pc));
  Kill(dst);
  emit(cond | B24 | B22 | B21 | 15*B16 | dst.code()*B12 |
       15*B8 | CLZ | src.code());
}


// Saturating instructions.

// Unsigned saturate.
void Assembler::usat(Register dst,
                     int satpos,
                     const Operand& src,
                     Condition cond) {
  using namespace arm;
  // v6 and above.
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  ASSERT(!dst.is(pc) && !src.rm_.is(pc));
  ASSERT((satpos >= 0) && (satpos <= 31));
  ASSERT((src.shift_op_ == ASR) || (src.shift_op_ == LSL));
  ASSERT(src.rs_.is(no_reg));
  Kill(dst);

  int sh = 0;
  if (src.shift_op_ == ASR) {
      sh = 1;
  }

  emit(cond | 0x6*B24 | 0xe*B20 | satpos*B16 | dst.code()*B12 |
       src.shift_imm_*B7 | sh*B6 | 0x1*B4 | src.rm_.code());
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
  using namespace arm;
  // v7 and above.
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  ASSERT(!dst.is(pc) && !src.is(pc));
  ASSERT((lsb >= 0) && (lsb <= 31));
  ASSERT((width >= 1) && (width <= (32 - lsb)));
  Kill(dst);

  emit(cond | 0xf*B23 | B22 | B21 | (width - 1)*B16 | dst.code()*B12 |
       lsb*B7 | B6 | B4 | src.code());
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
  using namespace arm;
  // v7 and above.
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  ASSERT(!dst.is(pc) && !src.is(pc));
  ASSERT((lsb >= 0) && (lsb <= 31));
  ASSERT((width >= 1) && (width <= (32 - lsb)));
  Kill(dst);

  emit(cond | 0xf*B23 | B21 | (width - 1)*B16 | dst.code()*B12 |
       lsb*B7 | B6 | B4 | src.code());
}


// Bit field clear.
// Sets #width adjacent bits at position #lsb in the destination register
// to zero, preserving the value of the other bits.
//   bfc dst, #lsb, #width
void Assembler::bfc(Register dst, int lsb, int width, Condition cond) {
  using namespace arm;
  // v7 and above.
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  ASSERT(!dst.is(pc));
  ASSERT((lsb >= 0) && (lsb <= 31));
  ASSERT((width >= 1) && (width <= (32 - lsb)));
  Kill(dst);

  int msb = lsb + width - 1;
  emit(cond | 0x1f*B22 | msb*B16 | dst.code()*B12 | lsb*B7 | B4 | 0xf);
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
  using namespace arm;
  // v7 and above.
  ASSERT(CpuFeatures::IsSupported(ARMv7));
  ASSERT(!dst.is(pc) && !src.is(pc));
  ASSERT((lsb >= 0) && (lsb <= 31));
  ASSERT((width >= 1) && (width <= (32 - lsb)));
  Kill(dst);

  int msb = lsb + width - 1;
  emit(cond | 0x1f*B22 | msb*B16 | dst.code()*B12 | lsb*B7 | B4 |
       src.code());
}

void Assembler::pkhbt(Register dst,
                      Register src1,
                      const Operand& src2,
                      Condition cond ) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.125.
  // cond(31-28) | 01101000(27-20) | Rn(19-16) |
  // Rd(15-12) | imm5(11-7) | 0(6) | 01(5-4) | Rm(3-0)
  ASSERT(!dst.is(pc));
  ASSERT(!src1.is(pc));
  ASSERT(!src2.rm().is(pc));
  ASSERT(!src2.rm().is(no_reg));
  ASSERT(src2.rs().is(no_reg));
  ASSERT((src2.shift_imm_ >= 0) && (src2.shift_imm_ <= 31));
  ASSERT(src2.shift_op() == LSL);
  emit(cond | 0x68*B20 | src1.code()*B16 | dst.code()*B12 |
       src2.shift_imm_*B7 | B4 | src2.rm().code());
}


void Assembler::pkhtb(Register dst,
                      Register src1,
                      const Operand& src2,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.125.
  // cond(31-28) | 01101000(27-20) | Rn(19-16) |
  // Rd(15-12) | imm5(11-7) | 1(6) | 01(5-4) | Rm(3-0)
  ASSERT(!dst.is(pc));
  ASSERT(!src1.is(pc));
  ASSERT(!src2.rm().is(pc));
  ASSERT(!src2.rm().is(no_reg));
  ASSERT(src2.rs().is(no_reg));
  ASSERT((src2.shift_imm_ >= 1) && (src2.shift_imm_ <= 32));
  ASSERT(src2.shift_op() == ASR);
  int asr = (src2.shift_imm_ == 32) ? 0 : src2.shift_imm_;
  emit(cond | 0x68*B20 | src1.code()*B16 | dst.code()*B12 |
       asr*B7 | B6 | B4 | src2.rm().code());
}


void Assembler::uxtb(Register dst,
                     const Operand& src,
                     Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.274.
  // cond(31-28) | 01101110(27-20) | 1111(19-16) |
  // Rd(15-12) | rotate(11-10) | 00(9-8)| 0111(7-4) | Rm(3-0)
  ASSERT(!dst.is(pc));
  ASSERT(!src.rm().is(pc));
  ASSERT(!src.rm().is(no_reg));
  ASSERT(src.rs().is(no_reg));
  ASSERT((src.shift_imm_ == 0) ||
         (src.shift_imm_ == 8) ||
         (src.shift_imm_ == 16) ||
         (src.shift_imm_ == 24));
  ASSERT(src.shift_op() == ROR);
  emit(cond | 0x6E*B20 | 0xF*B16 | dst.code()*B12 |
       ((src.shift_imm_ >> 1)&0xC)*B8 | 7*B4 | src.rm().code());
}


void Assembler::uxtab(Register dst,
                      Register src1,
                      const Operand& src2,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.271.
  // cond(31-28) | 01101110(27-20) | Rn(19-16) |
  // Rd(15-12) | rotate(11-10) | 00(9-8)| 0111(7-4) | Rm(3-0)
  ASSERT(!dst.is(pc));
  ASSERT(!src1.is(pc));
  ASSERT(!src2.rm().is(pc));
  ASSERT(!src2.rm().is(no_reg));
  ASSERT(src2.rs().is(no_reg));
  ASSERT((src2.shift_imm_ == 0) ||
         (src2.shift_imm_ == 8) ||
         (src2.shift_imm_ == 16) ||
         (src2.shift_imm_ == 24));
  ASSERT(src2.shift_op() == ROR);
  emit(cond | 0x6E*B20 | src1.code()*B16 | dst.code()*B12 |
       ((src2.shift_imm_ >> 1) &0xC)*B8 | 7*B4 | src2.rm().code());
}


void Assembler::uxtb16(Register dst,
                       const Operand& src,
                       Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.275.
  // cond(31-28) | 01101100(27-20) | 1111(19-16) |
  // Rd(15-12) | rotate(11-10) | 00(9-8)| 0111(7-4) | Rm(3-0)
  ASSERT(!dst.is(pc));
  ASSERT(!src.rm().is(pc));
  ASSERT(!src.rm().is(no_reg));
  ASSERT(src.rs().is(no_reg));
  ASSERT((src.shift_imm_ == 0) ||
         (src.shift_imm_ == 8) ||
         (src.shift_imm_ == 16) ||
         (src.shift_imm_ == 24));
  ASSERT(src.shift_op() == ROR);
  emit(cond | 0x6C*B20 | 0xF*B16 | dst.code()*B12 |
       ((src.shift_imm_ >> 1)&0xC)*B8 | 7*B4 | src.rm().code());
}

// Status register access instructions.
void Assembler::mrs(Register dst, SRegister s, Condition cond) {
  using namespace arm;
  ASSERT(!dst.is(pc));
  Kill(dst);

  emit(cond | B24 | s | 15*B16 | dst.code()*B12);
}


void Assembler::msr(SRegisterFieldMask fields, const Operand& src,
                    Condition cond) {
  ASSERT(fields >= B16 && fields < B20);  // at least one field set
  using namespace arm;
  Instr instr;
  if (!src.rm_.is_valid()) {
    // Immediate.
    uint32_t rotate_imm;
    uint32_t immed_8;
    if (src.must_output_reloc_info(this) ||
        !fits_shifter(src.imm32_, &rotate_imm, &immed_8, NULL)) {
      // Immediate operand cannot be encoded, load it first to register ip.
      RecordRelocInfo(src.rmode_, src.imm32_);
      ldr(ip, MemOperand(pc, 0), cond);
      msr(fields, Operand(ip), cond);
      return;
    }
    instr = I | rotate_imm*B8 | immed_8;
  } else {
    ASSERT(!src.rs_.is_valid() && src.shift_imm_ == 0);  // only rm allowed
    instr = src.rm_.code();
  }
  emit(cond | instr | B24 | B21 | fields | 15*B12);
}


// Load/Store instructions.
void Assembler::ldr(Register dst, const MemOperand& src, Condition cond) {
  if (dst.is(pc)) {
    positions_recorder()->WriteRecordedPositions();
  }
  using namespace arm;
  Kill(dst);
  addrmod2(cond | B26 | L, dst, src);
}


void Assembler::str(Register src, const MemOperand& dst, Condition cond) {
  addrmod2(cond | B26, src, dst);
}


void Assembler::ldrb(Register dst, const MemOperand& src, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod2(cond | B26 | B | L, dst, src);
}


void Assembler::strb(Register src, const MemOperand& dst, Condition cond) {
  using namespace arm;
  addrmod2(cond | B26 | B, src, dst);
}


void Assembler::ldrh(Register dst, const MemOperand& src, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod3(cond | L | B7 | H | B4, dst, src);
}


void Assembler::strh(Register src, const MemOperand& dst, Condition cond) {
  using namespace arm;
  addrmod3(cond | B7 | H | B4, src, dst);
}


void Assembler::ldrsb(Register dst, const MemOperand& src, Condition cond) {
  using namespace arm;
  Kill(dst);
  addrmod3(cond | L | B7 | S6 | B4, dst, src);
}


void Assembler::ldrsh(Register dst, const MemOperand& src, Condition cond) {
  using namespace arm;
  addrmod3(cond | L | B7 | S6 | H | B4, dst, src);
}


void Assembler::ldrd(Register dst1, Register dst2,
                     const MemOperand& src, Condition cond) {
  ASSERT(src.rm().is(no_reg));
  ASSERT(IsEnabled(ARMv7));
  ASSERT(!dst1.is(lr));  // r14.
  ASSERT_EQ(0, dst1.code() % 2);
  ASSERT_EQ(dst1.code() + 1, dst2.code());
  Kill(dst1);
  Kill(dst2);
  addrmod3(cond | B7 | B6 | B4, dst1, src);
}


void Assembler::strd(Register src1, Register src2,
                     const MemOperand& dst, Condition cond) {
  ASSERT(dst.rm().is(no_reg));
  ASSERT(!src1.is(lr));  // r14.
  ASSERT_EQ(0, src1.code() % 2);
  ASSERT_EQ(src1.code() + 1, src2.code());
  ASSERT(IsEnabled(ARMv7));
  addrmod3(cond | B7 | B6 | B5 | B4, src1, dst);
}


// Preload instructions.
void Assembler::pld(const MemOperand& address) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.128.
  // 1111(31-28) | 0111(27-24) | U(23) | R(22) | 01(21-20) | Rn(19-16) |
  // 1111(15-12) | imm5(11-07) | type(6-5) | 0(4)| Rm(3-0) |
  ASSERT(address.rm().is(no_reg));
  ASSERT(address.am() == Offset);
  int U = B23;
  int offset = address.offset();
  if (offset < 0) {
    offset = -offset;
    U = 0;
  }
  ASSERT(offset < 4096);
  emit(kSpecialCondition | B26 | B24 | U | B22 | B20 | address.rn().code()*B16 |
       0xf*B12 | offset);
}

// Load/Store multiple instructions.
void Assembler::ldm(BlockAddrMode am,
                    Register base,
                    RegList dst,
                    Condition cond) {
  using namespace arm;
  // ABI stack constraint: ldmxx base, {..sp..}  base != sp  is not restartable.
  ASSERT(base.is(sp) || (dst & sp.bit()) == 0);
  Kill(dst);
  addrmod4(cond | B27 | am | L, base, dst);

  // Emit the constant pool after a function return implemented by ldm ..{..pc}.
  if (cond == al && (dst & pc.bit()) != 0) {
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
  using namespace arm;
  addrmod4(cond | B27 | am, base, src);
}


void Assembler::bkpt(uint32_t imm) {  // v5 and above
  using namespace arm;
  ASSERT(is_uint16(imm));
  emit(al | B24 | B21 | (imm >> 4)*B8 | BKPT | (imm & 0xf));
}


void Assembler::svc(uint32_t imm, Condition cond) {
  using namespace arm;
  ASSERT(is_uint24(imm));
  emit(cond | 15*B24 | imm);
}


// Pseudo instructions.
void Assembler::nop(int type) {
  // ARMv6{K/T2} and v7 have an actual NOP instruction but it serializes
  // some of the CPU's pipeline and has to issue. Older ARM chips simply used
  // MOV Rx, Rx as NOP and it performs better even in newer CPUs.
  // We therefore use MOV Rx, Rx, even on newer CPUs, and use Rx to encode
  // a type.
  ASSERT(0 <= type && type <= 14);  // mov pc, pc isn't a nop.
  emit(al | 13*B21 | type*B12 | type);
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
