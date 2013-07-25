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
//

#include "v8.h"

#if defined(V8_TARGET_ARCH_ARM)

#include "arm/assembler-arm-inl.h"
#include "serialize.h"

namespace v8 {
namespace internal {

// Coprocessor instructions.
void Assembler::emit_coproc(const Instr instr, const Condition cond) {
#ifndef USE_THUMB
  emit(cond | instr);
#else
  using namespace thumb32;
  if (cond == kSpecialCondition) {
    emit32(T32Prefix11 | instr, al);
  } else {
    emit32(T32Prefix01 | instr, cond);
  }
#endif
}

void Assembler::coproc_addrmod(Instr instr, CRegister crd, const MemOperand& x, Condition cond) {
  // Unindexed addressing is not encoded by this function.
  using namespace arm;
  ASSERT_EQ((B27 | B26),
            (instr & ~(kCondMask | kCoprocessorMask | P | U | N | W | L)));
  ASSERT(x.rn_.is_valid() && !x.rm_.is_valid());
  int am = x.am_;
  int offset_8 = x.offset_;
  ASSERT((offset_8 & 3) == 0);  // offset must be an aligned word offset
  offset_8 >>= 2;
  if (offset_8 < 0) {
    offset_8 = -offset_8;
    am ^= U;
  }
  ASSERT(is_uint8(offset_8));  // unsigned word offset must fit in a byte
  ASSERT((am & (P|W)) == P || !x.rn_.is(pc));  // no pc base with writeback

  // Post-indexed addressing requires W == 1; different than in addrmod2/3.
  if ((am & P) == 0)
    am |= W;

  ASSERT(offset_8 >= 0);  // no masking needed
  emit_coproc(instr | am | x.rn_.code()*B16 | crd.code()*B12 | offset_8, cond);
}

void Assembler::cdp(Coprocessor coproc,
                    int opcode_1,
                    CRegister crd,
                    CRegister crn,
                    CRegister crm,
                    int opcode_2,
                    Condition cond) {
  ASSERT(is_uint4(opcode_1) && is_uint3(opcode_2));
  Instr instr =  B27 | B26 | B25 | (opcode_1 & 15)*B20 | crn.code()*B16 |
                 crd.code()*B12 | coproc*B8 | (opcode_2 & 7)*B5 | crm.code();
  emit_coproc(instr, cond);
}


void Assembler::cdp2(Coprocessor coproc,
                     int opcode_1,
                     CRegister crd,
                     CRegister crn,
                     CRegister crm,
                     int opcode_2) {  // v5 and above
  cdp(coproc, opcode_1, crd, crn, crm, opcode_2, kSpecialCondition);
}


void Assembler::mcr(Coprocessor coproc,
                    int opcode_1,
                    Register rd,
                    CRegister crn,
                    CRegister crm,
                    int opcode_2,
                    Condition cond) {
  ASSERT(is_uint3(opcode_1) && is_uint3(opcode_2));
  Instr instr =  B27 | B26 | B25 | (opcode_1 & 7)*B21 | crn.code()*B16 |
                 rd.code()*B12 | coproc*B8 | (opcode_2 & 7)*B5 | B4 | crm.code();
  emit_coproc(instr, cond);
}


void Assembler::mcr2(Coprocessor coproc,
                     int opcode_1,
                     Register rd,
                     CRegister crn,
                     CRegister crm,
                     int opcode_2) {  // v5 and above
  mcr(coproc, opcode_1, rd, crn, crm, opcode_2, kSpecialCondition);
}


void Assembler::mrc(Coprocessor coproc,
                    int opcode_1,
                    Register rd,
                    CRegister crn,
                    CRegister crm,
                    int opcode_2,
                    Condition cond) {
  ASSERT(is_uint3(opcode_1) && is_uint3(opcode_2));
  Instr instr = B27 | B26 | B25 | (opcode_1 & 7)*B21 | L | crn.code()*B16 |
                rd.code()*B12 | coproc*B8 | (opcode_2 & 7)*B5 | B4 | crm.code();
  emit_coproc(instr, cond);
}


void Assembler::mrc2(Coprocessor coproc,
                     int opcode_1,
                     Register rd,
                     CRegister crn,
                     CRegister crm,
                     int opcode_2) {  // v5 and above
  mrc(coproc, opcode_1, rd, crn, crm, opcode_2, kSpecialCondition);
}


void Assembler::ldc(Coprocessor coproc,
                    CRegister crd,
                    const MemOperand& src,
                    LFlag l,
                    Condition cond) {
  Instr instr = B27 | B26 | l | L | coproc*B8;
  coproc_addrmod(instr, crd, src, cond);
}


void Assembler::ldc(Coprocessor coproc,
                    CRegister crd,
                    Register rn,
                    int option,
                    LFlag l,
                    Condition cond) {
  // Unindexed addressing.
  ASSERT(is_uint8(option));
  Instr instr = B27 | B26 | U | l | L | rn.code()*B16 | crd.code()*B12 |
                coproc*B8 | (option & 255);
  emit_coproc(instr, cond);
}


void Assembler::ldc2(Coprocessor coproc,
                     CRegister crd,
                     const MemOperand& src,
                     LFlag l) {  // v5 and above
  ldc(coproc, crd, src, l, kSpecialCondition);
}


void Assembler::ldc2(Coprocessor coproc,
                     CRegister crd,
                     Register rn,
                     int option,
                     LFlag l) {  // v5 and above
  ldc(coproc, crd, rn, option, l, kSpecialCondition);
}


// Support for VFP.
void Assembler::emit_vfp(const Instr instr, const Condition cond) {
#ifndef USE_THUMB
  emit(cond | instr);
#else
  using namespace thumb32;
  emit32(T32Prefix | instr, cond);
#endif
}

void Assembler::vldr(const DwVfpRegister dst,
                     const Register base,
                     int offset,
                     const Condition cond) {
  // Ddst = MEM(Rbase + offset).
  // Instruction details available in ARM DDI 0406C.b, A8-924.
  // cond(31-28) | 1101(27-24)| U(23) | D(22) | 01(21-20) | Rbase(19-16) |
  // Vd(15-12) | 1011(11-8) | offset
  int u = 1;
  if (offset < 0) {
    offset = -offset;
    u = 0;
  }
  int vd, d;
  dst.split_code(&vd, &d);

  ASSERT(offset >= 0);
  if ((offset % 4) == 0 && (offset / 4) < 256) {
    Instr instr = 0xD*B24 | u*B23 | d*B22 | B20 | base.code()*B16 | vd*B12 |
         0xB*B8 | ((offset / 4) & 255);
    emit_vfp(instr, cond);
  } else {
    // Larger offsets must be handled by computing the correct address
    // in the ip register.
    ASSERT(!base.is(ip));
    if (u == 1) {
      add(ip, base, Operand(offset));
    } else {
      sub(ip, base, Operand(offset));
    }
    Instr instr = 0xD*B24 | d*B22 | B20 | ip.code()*B16 | vd*B12 | 0xB*B8;
    emit_vfp(instr, cond);
  }
}


void Assembler::vldr(const DwVfpRegister dst,
                     const MemOperand& operand,
                     const Condition cond) {
  ASSERT(!operand.rm().is_valid());
  ASSERT(operand.am_ == Offset);
  vldr(dst, operand.rn(), operand.offset(), cond);
}


void Assembler::vldr(const SwVfpRegister dst,
                     const Register base,
                     int offset,
                     const Condition cond) {
  // Sdst = MEM(Rbase + offset).
  // Instruction details available in ARM DDI 0406A, A8-628.
  // cond(31-28) | 1101(27-24)| U001(23-20) | Rbase(19-16) |
  // Vdst(15-12) | 1010(11-8) | offset
  int u = 1;
  if (offset < 0) {
    offset = -offset;
    u = 0;
  }
  int sd, d;
  dst.split_code(&sd, &d);
  ASSERT(offset >= 0);

  if ((offset % 4) == 0 && (offset / 4) < 256) {
    Instr instr =  u*B23 | d*B22 | 0xD1*B20 | base.code()*B16 | sd*B12 |
                   0xA*B8 | ((offset / 4) & 255);
    emit_vfp(instr, cond);
  } else {
    // Larger offsets must be handled by computing the correct address
    // in the ip register.
    ASSERT(!base.is(ip));
    if (u == 1) {
      add(ip, base, Operand(offset));
    } else {
      sub(ip, base, Operand(offset));
    }
    Instr instr = d*B22 | 0xD1*B20 | ip.code()*B16 | sd*B12 | 0xA*B8;
    emit_vfp(instr, cond);
  }
}


void Assembler::vldr(const SwVfpRegister dst,
                     const MemOperand& operand,
                     const Condition cond) {
  ASSERT(!operand.rm().is_valid());
  ASSERT(operand.am_ == Offset);
  vldr(dst, operand.rn(), operand.offset(), cond);
}


void Assembler::vstr(const DwVfpRegister src,
                     const Register base,
                     int offset,
                     const Condition cond) {
  // MEM(Rbase + offset) = Dsrc.
  // Instruction details available in ARM DDI 0406C.b, A8-1082.
  // cond(31-28) | 1101(27-24)| U(23) | D(22) | 00(21-20) | Rbase(19-16) |
  // Vd(15-12) | 1011(11-8) | (offset/4)
  int u = 1;
  if (offset < 0) {
    offset = -offset;
    u = 0;
  }
  ASSERT(offset >= 0);
  int vd, d;
  src.split_code(&vd, &d);

  if ((offset % 4) == 0 && (offset / 4) < 256) {
    Instr instr =  0xD*B24 | u*B23 | d*B22 | base.code()*B16 | vd*B12 | 0xB*B8 |
                   ((offset / 4) & 255);
    emit_vfp(instr, cond);
  } else {
    // Larger offsets must be handled by computing the correct address
    // in the ip register.
    ASSERT(!base.is(ip));
    if (u == 1) {
      add(ip, base, Operand(offset));
    } else {
      sub(ip, base, Operand(offset));
    }
    Instr instr = 0xD*B24 | d*B22 | ip.code()*B16 | vd*B12 | 0xB*B8;
    emit_vfp(instr, cond);
  }
}


void Assembler::vstr(const DwVfpRegister src,
                     const MemOperand& operand,
                     const Condition cond) {
  ASSERT(!operand.rm().is_valid());
  ASSERT(operand.am_ == Offset);
  vstr(src, operand.rn(), operand.offset(), cond);
}


void Assembler::vstr(const SwVfpRegister src,
                     const Register base,
                     int offset,
                     const Condition cond) {
  // MEM(Rbase + offset) = SSrc.
  // Instruction details available in ARM DDI 0406A, A8-786.
  // cond(31-28) | 1101(27-24)| U000(23-20) | Rbase(19-16) |
  // Vdst(15-12) | 1010(11-8) | (offset/4)
  int u = 1;
  if (offset < 0) {
    offset = -offset;
    u = 0;
  }
  int sd, d;
  src.split_code(&sd, &d);
  ASSERT(offset >= 0);
  if ((offset % 4) == 0 && (offset / 4) < 256) {
    Instr instr = u*B23 | d*B22 | 0xD0*B20 | base.code()*B16 | sd*B12 |
                  0xA*B8 | ((offset / 4) & 255);
    emit_vfp(instr, cond);
  } else {
    // Larger offsets must be handled by computing the correct address
    // in the ip register.
    ASSERT(!base.is(ip));
    if (u == 1) {
      add(ip, base, Operand(offset));
    } else {
      sub(ip, base, Operand(offset));
    }
    Instr instr = d*B22 | 0xD0*B20 | ip.code()*B16 | sd*B12 | 0xA*B8;
    emit_vfp(instr, cond);
  }
}


void Assembler::vstr(const SwVfpRegister src,
                     const MemOperand& operand,
                     const Condition cond) {
  ASSERT(!operand.rm().is_valid());
  ASSERT(operand.am_ == Offset);
  vstr(src, operand.rn(), operand.offset(), cond);
}


void  Assembler::vldm(BlockAddrMode am,
                      Register base,
                      DwVfpRegister first,
                      DwVfpRegister last,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-922.
  // cond(31-28) | 110(27-25)| PUDW1(24-20) | Rbase(19-16) |
  // first(15-12) | 1011(11-8) | (count * 2)
  ASSERT_LE(first.code(), last.code());
  ASSERT(am == ia || am == ia_w || am == db_w);
  ASSERT(!base.is(pc));

  int sd, d;
  first.split_code(&sd, &d);
  int count = last.code() - first.code() + 1;
  ASSERT(count <= 16);
  Instr instr = B27 | B26 | am | d*B22 | B20 | base.code()*B16 | sd*B12 |
                0xB*B8 | count*2;
  emit_vfp(instr, cond);
}


void  Assembler::vstm(BlockAddrMode am,
                      Register base,
                      DwVfpRegister first,
                      DwVfpRegister last,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-1080.
  // cond(31-28) | 110(27-25)| PUDW0(24-20) | Rbase(19-16) |
  // first(15-12) | 1011(11-8) | (count * 2)
  ASSERT_LE(first.code(), last.code());
  ASSERT(am == ia || am == ia_w || am == db_w);
  ASSERT(!base.is(pc));

  int sd, d;
  first.split_code(&sd, &d);
  int count = last.code() - first.code() + 1;
  ASSERT(count <= 16);
  Instr instr = B27 | B26 | am | d*B22 | base.code()*B16 | sd*B12 |
                0xB*B8 | count*2;
  emit_vfp(instr, cond);
}

void  Assembler::vldm(BlockAddrMode am,
                      Register base,
                      SwVfpRegister first,
                      SwVfpRegister last,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406A, A8-626.
  // cond(31-28) | 110(27-25)| PUDW1(24-20) | Rbase(19-16) |
  // first(15-12) | 1010(11-8) | (count/2)
  ASSERT_LE(first.code(), last.code());
  ASSERT(am == ia || am == ia_w || am == db_w);
  ASSERT(!base.is(pc));

  int sd, d;
  first.split_code(&sd, &d);
  int count = last.code() - first.code() + 1;
  Instr instr = B27 | B26 | am | d*B22 | B20 | base.code()*B16 | sd*B12 |
                0xA*B8 | count;
  emit_vfp(instr, cond);
}


void  Assembler::vstm(BlockAddrMode am,
                      Register base,
                      SwVfpRegister first,
                      SwVfpRegister last,
                      Condition cond) {
  // Instruction details available in ARM DDI 0406A, A8-784.
  // cond(31-28) | 110(27-25)| PUDW0(24-20) | Rbase(19-16) |
  // first(15-12) | 1011(11-8) | (count/2)
  ASSERT_LE(first.code(), last.code());
  ASSERT(am == ia || am == ia_w || am == db_w);
  ASSERT(!base.is(pc));

  int sd, d;
  first.split_code(&sd, &d);
  int count = last.code() - first.code() + 1;
  Instr instr = B27 | B26 | am | d*B22 | base.code()*B16 | sd*B12 |
                0xA*B8 | count;
  emit_vfp(instr, cond);
}

static void DoubleAsTwoUInt32(double d, uint32_t* lo, uint32_t* hi) {
  uint64_t i;
  memcpy(&i, &d, 8);

  *lo = i & 0xffffffff;
  *hi = i >> 32;
}

// Only works for little endian floating point formats.
// We don't support VFP on the mixed endian floating point platform.
static bool FitsVMOVDoubleImmediate(double d, uint32_t *encoding) {
  ASSERT(CpuFeatures::IsSupported(VFP3));

  // VMOV can accept an immediate of the form:
  //
  //  +/- m * 2^(-n) where 16 <= m <= 31 and 0 <= n <= 7
  //
  // The immediate is encoded using an 8-bit quantity, comprised of two
  // 4-bit fields. For an 8-bit immediate of the form:
  //
  //  [abcdefgh]
  //
  // where a is the MSB and h is the LSB, an immediate 64-bit double can be
  // created of the form:
  //
  //  [aBbbbbbb,bbcdefgh,00000000,00000000,
  //      00000000,00000000,00000000,00000000]
  //
  // where B = ~b.
  //

  uint32_t lo, hi;
  DoubleAsTwoUInt32(d, &lo, &hi);

  // The most obvious constraint is the long block of zeroes.
  if ((lo != 0) || ((hi & 0xffff) != 0)) {
    return false;
  }

  // Bits 62:55 must be all clear or all set.
  if (((hi & 0x3fc00000) != 0) && ((hi & 0x3fc00000) != 0x3fc00000)) {
    return false;
  }

  // Bit 63 must be NOT bit 62.
  if (((hi ^ (hi << 1)) & (0x40000000)) == 0) {
    return false;
  }

  // Create the encoded immediate in the form:
  //  [00000000,0000abcd,00000000,0000efgh]
  *encoding  = (hi >> 16) & 0xf;      // Low nybble.
  *encoding |= (hi >> 4) & 0x70000;   // Low three bits of the high nybble.
  *encoding |= (hi >> 12) & 0x80000;  // Top bit of the high nybble.

  return true;
}


void Assembler::vmov(const DwVfpRegister dst,
                     double imm,
                     const Register scratch) {
  uint32_t enc;
  if (CpuFeatures::IsSupported(VFP3) && FitsVMOVDoubleImmediate(imm, &enc)) {
    // The double can be encoded in the instruction.
    //
    // Dd = immediate
    // Instruction details available in ARM DDI 0406C.b, A8-936.
    // cond(31-28) | 11101(27-23) | D(22) | 11(21-20) | imm4H(19-16) |
    // Vd(15-12) | 101(11-9) | sz=1(8) | imm4L(3-0)
    int vd, d;
    dst.split_code(&vd, &d);
    Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | vd*B12 | 0x5*B9 | B8 | enc;
    emit_vfp(instr, al);
  } else if (FLAG_enable_vldr_imm) {
    // TODO(jfb) Temporarily turned off until we have constant blinding or
    //           some equivalent mitigation: an attacker can otherwise control
    //           generated data which also happens to be executable, a Very Bad
    //           Thing indeed.
    //           Blinding gets tricky because we don't have xor, we probably
    //           need to add/subtract without losing precision, which requires a
    //           cookie value that Lithium is probably better positioned to
    //           choose.
    //           We could also add a few peepholes here like detecting 0.0 and
    //           -0.0 and doing a vmov from the sequestered d14, forcing denorms
    //           to zero (we set flush-to-zero), and normalizing NaN values.
    //           We could also detect redundant values.
    //           The code could also randomize the order of values, though
    //           that's tricky because vldr has a limited reach. Furthermore
    //           it breaks load locality.
    RecordRelocInfo(imm);
    vldr(dst, MemOperand(pc, 0));
  } else {
    // Synthesise the double from ARM immediates.
    uint32_t lo, hi;
    DoubleAsTwoUInt32(imm, &lo, &hi);

    if (scratch.is(no_reg)) {
      if (dst.code() < 16) {
        // Move the low part of the double into the lower of the corresponsing S
        // registers of D register dst.
        mov(ip, Operand(lo));
        vmov(dst.low(), ip);

        // Move the high part of the double into the higher of the
        // corresponsing S registers of D register dst.
        mov(ip, Operand(hi));
        vmov(dst.high(), ip);
      } else {
        // D16-D31 does not have S registers, so move the low and high parts
        // directly to the D register using vmov.32.
        // Note: This may be slower, so we only do this when we have to.
        mov(ip, Operand(lo));
        vmov(dst, VmovIndexLo, ip);
        mov(ip, Operand(hi));
        vmov(dst, VmovIndexHi, ip);
      }
    } else {
      // Move the low and high parts of the double to a D register in one
      // instruction.
      mov(ip, Operand(lo));
      mov(scratch, Operand(hi));
      vmov(dst, ip, scratch);
    }
  }
}


void Assembler::vmov(const SwVfpRegister dst,
                     const SwVfpRegister src,
                     const Condition cond) {
  // Sd = Sm
  // Instruction details available in ARM DDI 0406B, A8-642.
  int sd, d, sm, m;
  dst.split_code(&sd, &d);
  src.split_code(&sm, &m);
  Instr instr = 0xE*B24 | d*B22 | 0xB*B20 | sd*B12 | 0xA*B8 | B6 | m*B5 | sm;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const DwVfpRegister dst,
                     const DwVfpRegister src,
                     const Condition cond) {
  // Dd = Dm
  // Instruction details available in ARM DDI 0406C.b, A8-938.
  // cond(31-28) | 11101(27-23) | D(22) | 11(21-20) | 0000(19-16) | Vd(15-12) |
  // 101(11-9) | sz=1(8) | 0(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vm, m;
  src.split_code(&vm, &m);
  Instr instr =  0x1D*B23 | d*B22 | 0x3*B20 | vd*B12 | 0x5*B9 | B8 | B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const DwVfpRegister dst,
                     const VmovIndex index,
                     const Register src,
                     const Condition cond) {
  // Dd[index] = Rt
  // Instruction details available in ARM DDI 0406C.b, A8-940.
  // cond(31-28) | 1110(27-24) | 0(23) | opc1=0index(22-21) | 0(20) |
  // Vd(19-16) | Rt(15-12) | 1011(11-8) | D(7) | opc2=00(6-5) | 1(4) | 0000(3-0)
  ASSERT(index.index == 0 || index.index == 1);
  int vd, d;
  dst.split_code(&vd, &d);
  Instr instr = 0xE*B24 | index.index*B21 | vd*B16 | src.code()*B12 | 0xB*B8 | d*B7 | B4;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const DwVfpRegister dst,
                     const Register src1,
                     const Register src2,
                     const Condition cond) {
  // Dm = <Rt,Rt2>.
  // Instruction details available in ARM DDI 0406C.b, A8-948.
  // cond(31-28) | 1100(27-24)| 010(23-21) | op=0(20) | Rt2(19-16) |
  // Rt(15-12) | 1011(11-8) | 00(7-6) | M(5) | 1(4) | Vm
  ASSERT(!src1.is(pc) && !src2.is(pc));
  int vm, m;
  dst.split_code(&vm, &m);
  Instr instr = 0xC*B24 | B22 | src2.code()*B16 | src1.code()*B12 | 0xB*B8 | m*B5 | B4 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const Register dst1,
                     const Register dst2,
                     const DwVfpRegister src,
                     const Condition cond) {
  // <Rt,Rt2> = Dm.
  // Instruction details available in ARM DDI 0406C.b, A8-948.
  // cond(31-28) | 1100(27-24)| 010(23-21) | op=1(20) | Rt2(19-16) |
  // Rt(15-12) | 1011(11-8) | 00(7-6) | M(5) | 1(4) | Vm
  ASSERT(!dst1.is(pc) && !dst2.is(pc));
  int vm, m;
  src.split_code(&vm, &m);
  Instr instr = 0xC*B24 | B22 | B20 | dst2.code()*B16 | dst1.code()*B12 | 0xB*B8 | m*B5 | B4 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const SwVfpRegister dst,
                     const Register src,
                     const Condition cond) {
  // Sn = Rt.
  // Instruction details available in ARM DDI 0406A, A8-642.
  // cond(31-28) | 1110(27-24)| 000(23-21) | op=0(20) | Vn(19-16) |
  // Rt(15-12) | 1010(11-8) | N(7)=0 | 00(6-5) | 1(4) | 0000(3-0)
  ASSERT(!src.is(pc));
  int sn, n;
  dst.split_code(&sn, &n);
  Instr instr = 0xE*B24 | sn*B16 | src.code()*B12 | 0xA*B8 | n*B7 | B4;
  emit_vfp(instr, cond);
}


void Assembler::vmov(const Register dst,
                     const SwVfpRegister src,
                     const Condition cond) {
  // Rt = Sn.
  // Instruction details available in ARM DDI 0406A, A8-642.
  // cond(31-28) | 1110(27-24)| 000(23-21) | op=1(20) | Vn(19-16) |
  // Rt(15-12) | 1010(11-8) | N(7)=0 | 00(6-5) | 1(4) | 0000(3-0)
  ASSERT(!dst.is(pc));
  int sn, n;
  src.split_code(&sn, &n);
  Instr instr =  0xE*B24 | B20 | sn*B16 | dst.code()*B12 | 0xA*B8 | n*B7 | B4;
  emit_vfp(instr, cond);
}


// Type of data to read from or write to VFP register.
// Used as specifier in generic vcvt instruction.
enum VFPType { S32, U32, F32, F64 };


static bool IsSignedVFPType(VFPType type) {
  switch (type) {
    case S32:
      return true;
    case U32:
      return false;
    default:
      UNREACHABLE();
      return false;
  }
}


static bool IsIntegerVFPType(VFPType type) {
  switch (type) {
    case S32:
    case U32:
      return true;
    case F32:
    case F64:
      return false;
    default:
      UNREACHABLE();
      return false;
  }
}


static bool IsDoubleVFPType(VFPType type) {
  switch (type) {
    case F32:
      return false;
    case F64:
      return true;
    default:
      UNREACHABLE();
      return false;
  }
}


// Split five bit reg_code based on size of reg_type.
//  32-bit register codes are Vm:M
//  64-bit register codes are M:Vm
// where Vm is four bits, and M is a single bit.
static void SplitRegCode(VFPType reg_type,
                         int reg_code,
                         int* vm,
                         int* m) {
  ASSERT((reg_code >= 0) && (reg_code <= 31));
  if (IsIntegerVFPType(reg_type) || !IsDoubleVFPType(reg_type)) {
    // 32 bit type.
    *m  = reg_code & 0x1;
    *vm = reg_code >> 1;
  } else {
    // 64 bit type.
    *m  = (reg_code & 0x10) >> 4;
    *vm = reg_code & 0x0F;
  }
}


// Encode vcvt.src_type.dst_type instruction.
static Instr EncodeVCVT(const VFPType dst_type,
                        const int dst_code,
                        const VFPType src_type,
                        const int src_code,
                        VFPConversionMode mode) {
  ASSERT(src_type != dst_type);
  int D, Vd, M, Vm;
  SplitRegCode(src_type, src_code, &Vm, &M);
  SplitRegCode(dst_type, dst_code, &Vd, &D);

  if (IsIntegerVFPType(dst_type) || IsIntegerVFPType(src_type)) {
    // Conversion between IEEE floating point and 32-bit integer.
    // Instruction details available in ARM DDI 0406B, A8.6.295.
    // cond(31-28) | 11101(27-23)| D(22) | 11(21-20) | 1(19) | opc2(18-16) |
    // Vd(15-12) | 101(11-9) | sz(8) | op(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
    ASSERT(!IsIntegerVFPType(dst_type) || !IsIntegerVFPType(src_type));

    int sz, opc2, op;

    if (IsIntegerVFPType(dst_type)) {
      opc2 = IsSignedVFPType(dst_type) ? 0x5 : 0x4;
      sz = IsDoubleVFPType(src_type) ? 0x1 : 0x0;
      op = mode;
    } else {
      ASSERT(IsIntegerVFPType(src_type));
      opc2 = 0x0;
      sz = IsDoubleVFPType(dst_type) ? 0x1 : 0x0;
      op = IsSignedVFPType(src_type) ? 0x1 : 0x0;
    }

    return (0xE*B24 | B23 | D*B22 | 0x3*B20 | B19 | opc2*B16 |
            Vd*B12 | 0x5*B9 | sz*B8 | op*B7 | B6 | M*B5 | Vm);
  } else {
    // Conversion between IEEE double and single precision.
    // Instruction details available in ARM DDI 0406B, A8.6.298.
    // cond(31-28) | 11101(27-23)| D(22) | 11(21-20) | 0111(19-16) |
    // Vd(15-12) | 101(11-9) | sz(8) | 1(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
    int sz = IsDoubleVFPType(src_type) ? 0x1 : 0x0;
    return (0xE*B24 | B23 | D*B22 | 0x3*B20 | 0x7*B16 |
            Vd*B12 | 0x5*B9 | sz*B8 | B7 | B6 | M*B5 | Vm);
  }
}


void Assembler::vcvt_f64_s32(const DwVfpRegister dst,
                             const SwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(F64, dst.code(), S32, src.code(), mode), cond);
}


void Assembler::vcvt_f32_s32(const SwVfpRegister dst,
                             const SwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(F32, dst.code(), S32, src.code(), mode), cond);
}


void Assembler::vcvt_f64_u32(const DwVfpRegister dst,
                             const SwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(F64, dst.code(), U32, src.code(), mode), cond);
}


void Assembler::vcvt_s32_f64(const SwVfpRegister dst,
                             const DwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(S32, dst.code(), F64, src.code(), mode), cond);
}


void Assembler::vcvt_u32_f64(const SwVfpRegister dst,
                             const DwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(U32, dst.code(), F64, src.code(), mode), cond);
}


void Assembler::vcvt_f64_f32(const DwVfpRegister dst,
                             const SwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(F64, dst.code(), F32, src.code(), mode), cond);
}


void Assembler::vcvt_f32_f64(const SwVfpRegister dst,
                             const DwVfpRegister src,
                             VFPConversionMode mode,
                             const Condition cond) {
  emit_vfp(EncodeVCVT(F32, dst.code(), F64, src.code(), mode), cond);
}

void Assembler::vcvt_f64_s32(const DwVfpRegister dst,
                             int fraction_bits,
                             const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-874.
  // cond(31-28) | 11101(27-23) | D(22) | 11(21-20) | 1010(19-16) | Vd(15-12) |
  // 101(11-9) | sf=1(8) | sx=1(7) | 1(6) | i(5) | 0(4) | imm4(3-0)
  ASSERT(fraction_bits > 0 && fraction_bits <= 32);
  ASSERT(CpuFeatures::IsSupported(VFP3));
  int vd, d;
  dst.split_code(&vd, &d);
  int i = ((32 - fraction_bits) >> 4) & 1;
  int imm4 = (32 - fraction_bits) & 0xf;
  Instr instr =  0xE*B24 | B23 | d*B22 | 0x3*B20 | B19 | 0x2*B16 | vd*B12 | 0x5*B9 | B8 | B7 | B6 | i*B5 | imm4;
  emit_vfp(instr, cond);
}

void Assembler::vneg(const DwVfpRegister dst,
                     const DwVfpRegister src,
                     const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-968.
  // cond(31-28) | 11101(27-23) | D(22) | 11(21-20) | 0001(19-16) | Vd(15-12) |
  // 101(11-9) | sz=1(8) | 0(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vm, m;
  src.split_code(&vm, &m);

  Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | B16 | vd*B12 | 0x5*B9 | B8 | B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vabs(const DwVfpRegister dst,
                     const DwVfpRegister src,
                     const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-524.
  // cond(31-28) | 11101(27-23) | D(22) | 11(21-20) | 0000(19-16) | Vd(15-12) |
  // 101(11-9) | sz=1(8) | 1(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vm, m;
  src.split_code(&vm, &m);
  Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | vd*B12 | 0x5*B9 | B8 | B7 | B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vadd(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Dd = vadd(Dn, Dm) double precision floating point addition.
  // Dd = D:Vd; Dm=M:Vm; Dn=N:Vm.
  // Instruction details available in ARM DDI 0406C.b, A8-830.
  // cond(31-28) | 11100(27-23)| D(22) | 11(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | 0(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1C*B23 | d*B22 | 0x3*B20 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vsub(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Dd = vsub(Dn, Dm) double precision floating point subtraction.
  // Dd = D:Vd; Dm=M:Vm; Dn=N:Vm.
  // Instruction details available in ARM DDI 0406C.b, A8-1086.
  // cond(31-28) | 11100(27-23)| D(22) | 11(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1C*B23 | d*B22 | 0x3*B20 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmul(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Dd = vmul(Dn, Dm) double precision floating point multiplication.
  // Dd = D:Vd; Dm=M:Vm; Dn=N:Vm.
  // Instruction details available in ARM DDI 0406C.b, A8-960.
  // cond(31-28) | 11100(27-23)| D(22) | 10(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | 0(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1C*B23 | d*B22 | 0x2*B20 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmla(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-932.
  // cond(31-28) | 11100(27-23) | D(22) | 00(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | op=0(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr =  0x1C*B23 | d*B22 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vmls(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-932.
  // cond(31-28) | 11100(27-23) | D(22) | 00(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | op=1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1C*B23 | d*B22 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vdiv(const DwVfpRegister dst,
                     const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // Dd = vdiv(Dn, Dm) double precision floating point division.
  // Dd = D:Vd; Dm=M:Vm; Dn=N:Vm.
  // Instruction details available in ARM DDI 0406C.b, A8-882.
  // cond(31-28) | 11101(27-23)| D(22) | 00(21-20) | Vn(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | N(7) | 0(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vn, n;
  src1.split_code(&vn, &n);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1D*B23 | d*B22 | vn*B16 | vd*B12 | 0x5*B9 | B8 | n*B7 | m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vcmp(const DwVfpRegister src1,
                     const DwVfpRegister src2,
                     const Condition cond) {
  // vcmp(Dd, Dm) double precision floating point comparison.
  // Instruction details available in ARM DDI 0406C.b, A8-864.
  // cond(31-28) | 11101(27-23)| D(22) | 11(21-20) | 0100(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | E=0(7) | 1(6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  src1.split_code(&vd, &d);
  int vm, m;
  src2.split_code(&vm, &m);
  Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | 0x4*B16 | vd*B12 | 0x5*B9 | B8 | B6 |
       m*B5 | vm;
  emit_vfp(instr, cond);
}


void Assembler::vcmp(const DwVfpRegister src1,
                     const double src2,
                     const Condition cond) {
  // vcmp(Dd, #0.0) double precision floating point comparison.
  // Instruction details available in ARM DDI 0406C.b, A8-864.
  // cond(31-28) | 11101(27-23)| D(22) | 11(21-20) | 0101(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | E=0(7) | 1(6) | 0(5) | 0(4) | 0000(3-0)
  ASSERT(src2 == 0.0);
  int vd, d;
  src1.split_code(&vd, &d);
  Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | 0x5*B16 | vd*B12 | 0x5*B9 | B8 | B6;
  emit_vfp(instr, cond);
}


void Assembler::vmsr(Register dst, Condition cond) {
  // Instruction details available in ARM DDI 0406A, A8-652.
  // cond(31-28) | 1110 (27-24) | 1110(23-20)| 0001 (19-16) |
  // Rt(15-12) | 1010 (11-8) | 0(7) | 00 (6-5) | 1(4) | 0000(3-0)
  Instr instr = 0xE*B24 | 0xE*B20 | B16 | dst.code()*B12 | 0xA*B8 | B4;
  emit_vfp(instr, cond);
}


void Assembler::vmrs(Register dst, Condition cond) {
  // Instruction details available in ARM DDI 0406A, A8-652.
  // cond(31-28) | 1110 (27-24) | 1111(23-20)| 0001 (19-16) |
  // Rt(15-12) | 1010 (11-8) | 0(7) | 00 (6-5) | 1(4) | 0000(3-0)
  Instr instr = 0xE*B24 | 0xF*B20 | B16 | dst.code()*B12 | 0xA*B8 | B4;
  emit_vfp(instr, cond);
}


void Assembler::vsqrt(const DwVfpRegister dst,
                      const DwVfpRegister src,
                      const Condition cond) {
  // Instruction details available in ARM DDI 0406C.b, A8-1058.
  // cond(31-28) | 11101(27-23)| D(22) | 11(21-20) | 0001(19-16) |
  // Vd(15-12) | 101(11-9) | sz=1(8) | 11(7-6) | M(5) | 0(4) | Vm(3-0)
  int vd, d;
  dst.split_code(&vd, &d);
  int vm, m;
  src.split_code(&vm, &m);
  Instr instr = 0x1D*B23 | d*B22 | 0x3*B20 | B16 | vd*B12 | 0x5*B9 | B8 | 0x3*B6 | m*B5 | vm;
  emit_vfp(instr, cond);
}

// Support for NEON.
void Assembler::emit_neon(const Instr instr) {
#ifndef USE_THUMB
  emit(instr | 0xFU*B28 | 4*B24);
#else
  emit32(instr | 0xFU * H12 | 0x9 * H8, al);
#endif
}

void Assembler::vld1(NeonSize size,
                     const NeonListOperand& dst,
                     const NeonMemOperand& src) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.320.
  // 1111(31-28) | 01000(27-23) | D(22) | 10(21-20) | Rn(19-16) |
  // Vd(15-12) | type(11-8) | size(7-6) | align(5-4) | Rm(3-0)
  ASSERT(CpuFeatures::IsSupported(NEON));
  int vd, d;
  dst.base().split_code(&vd, &d);
  Instr instr = d*B22 | 2*B20 | src.rn().code()*B16 | vd*B12 |
                dst.type()*B8 | size*B6 | src.align()*B4 | src.rm().code();
  emit_neon(instr);
}


void Assembler::vst1(NeonSize size,
                     const NeonListOperand& src,
                     const NeonMemOperand& dst) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.404.
  // 1111(31-28) | 01000(27-23) | D(22) | 00(21-20) | Rn(19-16) |
  // Vd(15-12) | type(11-8) | size(7-6) | align(5-4) | Rm(3-0)
  ASSERT(CpuFeatures::IsSupported(NEON));
  int vd, d;
  src.base().split_code(&vd, &d);
  Instr instr = d*B22 | dst.rn().code()*B16 | vd*B12 | src.type()*B8 |
                size*B6 | dst.align()*B4 | dst.rm().code();
  emit_neon(instr);
}


void Assembler::vmovl(NeonDataType dt, QwNeonRegister dst, DwVfpRegister src) {
  // Instruction details available in ARM DDI 0406C.b, A8.8.346.
  // 1111(31-28) | 001(27-25) | U(24) | 1(23) | D(22) | imm3(21-19) |
  // 000(18-16) | Vd(15-12) | 101000(11-6) | M(5) | 1(4) | Vm(3-0)
  ASSERT(CpuFeatures::IsSupported(NEON));
  int vd, d;
  dst.split_code(&vd, &d);
  int vm, m;
  src.split_code(&vm, &m);
  Instr instr =  B23 | d*B22 |
                (dt & NeonDataTypeSizeMask)*B19 | vd*B12 | 0xA*B8 | m*B5 | B4 | vm;
#ifndef USE_THUMB
  instr |= 0xFU*B28 | B25 | (dt & NeonDataTypeUMask);
  emit(instr);
#else
  instr |= 0xEFU * H8 | ((dt & NeonDataTypeUMask) != 0) * H12;
  emit32(instr, al);
#endif
}
} }  // namespace v8::internal

#endif
