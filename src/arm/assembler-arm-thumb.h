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

// Thumb instruction generation

#ifndef V8_ARM_ASSEMBLER_ARM_THUMB_H_
#define V8_ARM_ASSEMBLER_ARM_THUMB_H_

// IT blocks management
int it_instr_pos_;  // Current IT instruction position, -1 if none
int it_size_;       // Number of instuctions in IT block so far

void fold_rm(Register r, const MemOperand& x, Condition cond = al);
void fold_rm(Register r, const Operand& x, Condition cond = al);

struct ThumbAddrMod1Options {
  thumb16::spec_data::Opcode spec_data_op;
  thumb16::data_proc1::Opcode data_proc1_op[6];
  thumb16::data_proc2::Opcode data_proc2_op;
  thumb32::mod_imm::Opcode mod_imm_op;
  thumb32::plain_imm::Opcode plain_imm_op;
  thumb32::shifted_reg::Opcode shifted_reg_op;
  bool is_move;
};

bool addrmod1_16_1(const ThumbAddrMod1Options& o, Register dst, Register src1,
                 const Operand& src2, SBit s, Condition cond);
bool addrmod1_16_2(const ThumbAddrMod1Options& o, Register dst, Register src1,
                 const Operand& src2, SBit s, Condition cond);
bool addrmod1_16_3(const ThumbAddrMod1Options& o, Register dst, Register src1,
                 const Operand& src2, SBit s, Condition cond);
bool addrmod1_32(const ThumbAddrMod1Options& o, Register dst, Register src1,
                 const Operand& src2, SBit s, Condition cond);
bool addrmod1(const ThumbAddrMod1Options& o, Register dst, Register src1,
              const Operand& src2, SBit s, Condition cond, bool ret_on_failure = false);

// choose the shorter of the two addrmod1 possibilities of immediate encodings
// and vs bic, add vs sub, mov vs mvn, etc
void choose_imm(ThumbAddrMod1Options o1, ThumbAddrMod1Options o2,
    Register dst, Register src1, const Operand& src2_1, const Operand& src2_2,
    SBit s, Condition cond);

void addrmod23(thumb16::mem::Opcode op16[], thumb32::mem::Opcode op, Register rt, const MemOperand& x, Condition cond, bool is_load);
void addrmod3d(thumb32::mem_dual::Opcode op, Register r1, Register r2, const MemOperand& x, Condition cond);
void addrmod4(thumb16::mem_multi::Opcode op16_1, thumb16::mem_multi::Opcode op16_2,
              thumb32::mem_multi::Opcode op32, BlockAddrMode am, Register base, RegList regs, Condition cond);

void thumb_shift(ShiftOp shift_op, Register dst, Register src1, const Operand& x, SBit s, Condition cond);
void thumb_bit_field(thumb32::plain_imm::Opcode op, Register dst, Register src,
                     int lsb, int width, Condition cond);
void thumb_mul_long(thumb32::muldiv::Opcode op, Register dstL, Register dstH,
                    Register src1, Register src2, SBit s, Condition cond);
void thumb_mul(thumb16::data_proc2::Opcode op1, thumb32::mul::Opcode op2,
               Register dst, Register src1, Register src2, Register srcA, SBit s, Condition cond);

// Thumb has a lot of non-uniform constaints which a tedious to check,
// we group these constraint verifiers in the following instruction selection routines.
//
// A6.3.1 Data-processing (modified immediate)
bool match(Instr32 *instr, thumb32::mod_imm::Opcode op, Register rd, Register rn, const Operand& x, SBit s);
// A6.3.3 Data-processing (plain binary immediate)
bool match(Instr32 *instr, thumb32::plain_imm::Opcode op, Register rd, Register rn, const Operand& x, SBit s);
bool match(Instr32* instr, thumb32::plain_imm::Opcode op, Register rd, const Operand& x, int satpos);
bool match(Instr32 *instr, thumb32::plain_imm::Opcode op, Register rd, Register rn, int lsb, int width);
// A6.3.11 Data-processing (shifted register)
bool match(Instr32* instr, thumb32::shifted_reg::Opcode op, Register rd, Register rn, const Operand& x, SBit s);
// A6.3.12 Data-processing (register)
bool match(Instr32* instr, ShiftOp shift_op, Register rd, Register rn, const Operand& x, SBit s);
bool match(Instr32* instr, thumb32::data_proc::Opcode op, Register rd, Register rn, const Operand& x, SBit s);
// Single item load-store
bool match(Instr32* instr, thumb32::mem::Opcode op, Register rt, const MemOperand& x);
// A6.3.16 Multiply, multiply accumulate, and absolute difference
bool match(Instr32* instr, thumb32::mul::Opcode op, Register rd, Register rn, Register rm, Register ra);
// A6.3.17 Long multiply, long multiply accumulate, and divide
bool match(Instr32* instr, thumb32::muldiv::Opcode op, Register rd, Register rn, Register rm);
bool match(Instr32* instr, thumb32::muldiv::Opcode op, Register rdlo, Register rdhi, Register rn, Register rm);
// A6.3.6 Load/store dual, load/store exclusive, table branch
bool match(Instr32* instr, thumb32::mem_dual::Opcode op, Register rt1, Register rt2, const MemOperand& x);
// A6.3.5 Load/store multiple
bool match(Instr32* instr, thumb32::mem_multi::Opcode op, BlockAddrMode am, Register rn, RegList rl);
// A6.3.15 Miscellaneous operations
bool match(Instr32* instr, thumb32::misc::Opcode op, Register rd, Register rm);
// Control 32bit
bool match(Instr32* instr, thumb32::control::Opcode op, const Operand& x, Condition cond);
// A8.8.18 Control 16-bit (unconditional branch, conditional branch)
bool match(Instr16* instr, thumb16::control::Opcode op, const Operand& x, Condition cond);
// A6.2.1 Shift (immediate), add, subtract, move, and compare
bool match(Instr16* instr, bool* it_effect,
           thumb16::data_proc1::Opcode op, Register rd, Register rn, const Operand& x, SBit s, Condition cond);
// A6.2.3 Special data instructions and branch and exchange
bool match(Instr16* instr, bool* it_effect,
           thumb16::spec_data::Opcode op, Register rd, Register rn, const Operand& x, SBit s);
// A6.2.2 Data-processing
bool match(Instr16* instr, bool* it_effect, thumb16::data_proc2::Opcode op,
           Register rd, Register rn, const Operand& x, SBit s, Condition cond);
// Handle shifts from A6.2.2
bool match(Instr16* instr, bool* it_effect, ShiftOp shift_op, Register rd,
           Register rn, const Operand& x, SBit s, Condition cond);
// A6.2.4 Load/store single data item
bool match(Instr16* instr, thumb16::mem::Opcode op, Register rt, const MemOperand& x);
// A8.8.199 STM (STMIA, STMEA), A8.8.57 LDM/LDMIA/LDMFD (Thumb), A8.8.133 PUSH, A8.8.131 POP (Thumb)
bool match(Instr16* instr, thumb16::mem_multi::Opcode op, BlockAddrMode am, Register rn, RegList rl);

protected:
// Emit 32-bit thumb instruction
inline void emit32(Instr32 x, Condition cond = al);
// Emit generic 16-bit thumb instruction
inline void emit16(Instr16 x, SBit s = LeaveCC, Condition cond = al);
// Use this one for 16-bit forms that do not require IT blocks to
// suppress the setting of CC flags
inline void emit16(Instr16 x, Condition cond = al);

public:

int it_size() { return it_size_; }
Condition it_cond() {
  if (it_instr_pos_ != -1) {
    ASSERT(it_size_ > 0);
    Instr16 instr = instr16_at(it_instr_pos_);
    Condition it_cond = thumb16::GetITCondition(instr);
    return it_cond;
  }
  return kSpecialCondition;
}

// Emit it instruction
void it(Condition cond, int16_t mask = 0);
// Start an IT block, if necessary
inline void cond(Condition c, bool expand_block);
inline void cond32(Condition c, bool expand_block);
// Close IT if open
inline void flush_cond();

virtual void enter_predictable_mode() {
  // When starting a predictable size mode close the IT block to break dependence on
  // the previously emitted code
  flush_cond();
}

void b_long(Label* L, Condition cond = al)  {
  b_long(branch_offset(L, cond == al), cond, L->is_bound());
}
void b_long(Condition cond, Label* L)  {
  b_long(branch_offset(L, cond == al), cond, L->is_bound());
}

void b_long(int branch_offset, Condition cond = al, bool offset_is_final = false);

Instr instr_at(int pos) { return *reinterpret_cast<Instr*>(buffer_ + pos); }
void instr_at_put(int pos, Instr instr) {
  *reinterpret_cast<Instr*>(buffer_ + pos) = instr;
}
static Instr instr_at(byte* pc) { return *reinterpret_cast<Instr*>(pc); }
static void instr_at_put(byte* pc, Instr instr) {
  *reinterpret_cast<Instr*>(pc) = instr;
}


// Read/patch instructions
// thumb 32bit
static Instr32 swap32(Instr32 x) {
 return (static_cast<uint32_t>(x) >> 16) | (static_cast<uint32_t>(x) << 16);
}
Instr32 instr32_at(int pos) { return swap32(*reinterpret_cast<Instr32*>(buffer_ + pos)); }
void instr32_at_put(int pos, Instr32 instr) {
  *reinterpret_cast<Instr32*>(buffer_ + pos) = swap32(instr);
}
static Instr32 instr32_at(byte* pc) { return swap32(*reinterpret_cast<Instr32*>(pc)); }
static void instr32_at_put(byte* pc, Instr32 instr) {
  *reinterpret_cast<Instr32*>(pc) = swap32(instr);
}
// thumb 16bit
Instr16 instr16_at(int pos) { return *reinterpret_cast<Instr16*>(buffer_ + pos); }
void instr16_at_put(int pos, Instr16 instr) {
  *reinterpret_cast<Instr16*>(buffer_ + pos) = instr;
}
static Instr16 instr16_at(byte* pc) { return *reinterpret_cast<Instr16*>(pc); }
static void instr16_at_put(byte* pc, Instr16 instr) {
  *reinterpret_cast<Instr16*>(pc) = instr;
}

#endif
