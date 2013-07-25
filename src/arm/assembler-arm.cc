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

#if V8_TARGET_ARCH_ARM

#include "arm/assembler-arm-inl.h"
#include "serialize.h"

namespace v8 {
namespace internal {

#ifdef DEBUG
bool CpuFeatures::initialized_ = false;
#endif
unsigned CpuFeatures::supported_ = 0;
unsigned CpuFeatures::found_by_runtime_probing_only_ = 0;
unsigned CpuFeatures::cache_line_size_ = 64;


ExternalReference ExternalReference::cpu_features() {
  ASSERT(CpuFeatures::initialized_);
  return ExternalReference(&CpuFeatures::supported_);
}


// Get the CPU features enabled by the build. For cross compilation the
// preprocessor symbols CAN_USE_ARMV7_INSTRUCTIONS and CAN_USE_VFP3_INSTRUCTIONS
// can be defined to enable ARMv7 and VFPv3 instructions when building the
// snapshot.
static unsigned CpuFeaturesImpliedByCompiler() {
  unsigned answer = 0;
#ifdef CAN_USE_ARMV7_INSTRUCTIONS
  if (FLAG_enable_armv7) {
    answer |= 1u << ARMv7;
  }
#endif  // CAN_USE_ARMV7_INSTRUCTIONS
#ifdef CAN_USE_VFP3_INSTRUCTIONS
  if (FLAG_enable_vfp3) {
    answer |= 1u << VFP3 | 1u << ARMv7;
  }
#endif  // CAN_USE_VFP3_INSTRUCTIONS
#ifdef CAN_USE_VFP32DREGS
  if (FLAG_enable_32dregs) {
    answer |= 1u << VFP32DREGS;
  }
#endif  // CAN_USE_VFP32DREGS
  if ((answer & (1u << ARMv7)) && FLAG_enable_unaligned_accesses) {
    answer |= 1u << UNALIGNED_ACCESSES;
  }

  return answer;
}


const char* DwVfpRegister::AllocationIndexToString(int index) {
  ASSERT(index >= 0 && index < NumAllocatableRegisters());
  ASSERT(kScratchDoubleReg.code() - kDoubleRegZero.code() ==
         kNumReservedRegisters - 1);
  if (index >= kDoubleRegZero.code())
    index += kNumReservedRegisters;

  return VFPRegisters::Name(index, true);
}


void CpuFeatures::Probe() {
  uint64_t standard_features = static_cast<unsigned>(
      OS::CpuFeaturesImpliedByPlatform()) | CpuFeaturesImpliedByCompiler();
  ASSERT(supported_ == 0 || supported_ == standard_features);
#ifdef DEBUG
  initialized_ = true;
#endif

  // Get the features implied by the OS and the compiler settings. This is the
  // minimal set of features which is also alowed for generated code in the
  // snapshot.
  supported_ |= standard_features;

  if (Serializer::enabled()) {
    // No probing for features if we might serialize (generate snapshot).
    printf("   ");
    PrintFeatures();
    return;
  }

#ifndef __arm__
  // For the simulator=arm build, use VFP when FLAG_enable_vfp3 is
  // enabled. VFPv3 implies ARMv7, see ARM DDI 0406B, page A1-6.
  if (FLAG_enable_vfp3) {
    supported_ |=
        static_cast<uint64_t>(1) << VFP3 |
        static_cast<uint64_t>(1) << ARMv7;
  }
  if (FLAG_enable_neon) {
    supported_ |= 1u << NEON;
  }
  // For the simulator=arm build, use ARMv7 when FLAG_enable_armv7 is enabled
  if (FLAG_enable_armv7) {
    supported_ |= static_cast<uint64_t>(1) << ARMv7;
  }

  if (FLAG_enable_sudiv) {
    supported_ |= static_cast<uint64_t>(1) << SUDIV;
  }

  if (FLAG_enable_movw_movt) {
    supported_ |= static_cast<uint64_t>(1) << MOVW_MOVT_IMMEDIATE_LOADS;
  }

  if (FLAG_enable_32dregs) {
    supported_ |= static_cast<uint64_t>(1) << VFP32DREGS;
  }

  if (FLAG_enable_unaligned_accesses) {
    supported_ |= static_cast<uint64_t>(1) << UNALIGNED_ACCESSES;
  }

#else  // __arm__
  // Probe for additional features not already known to be available.
  if (!IsSupported(VFP3) && FLAG_enable_vfp3 && OS::ArmCpuHasFeature(VFP3)) {
    // This implementation also sets the VFP flags if runtime
    // detection of VFP returns true. VFPv3 implies ARMv7, see ARM DDI
    // 0406B, page A1-6.
    found_by_runtime_probing_only_ |=
        static_cast<uint64_t>(1) << VFP3 |
        static_cast<uint64_t>(1) << ARMv7;
  }

  if (!IsSupported(NEON) && FLAG_enable_neon && OS::ArmCpuHasFeature(NEON)) {
    found_by_runtime_probing_only_ |= 1u << NEON;
  }

  if (!IsSupported(ARMv7) && FLAG_enable_armv7 && OS::ArmCpuHasFeature(ARMv7)) {
    found_by_runtime_probing_only_ |= static_cast<uint64_t>(1) << ARMv7;
  }

  if (!IsSupported(SUDIV) && FLAG_enable_sudiv && OS::ArmCpuHasFeature(SUDIV)) {
    found_by_runtime_probing_only_ |= static_cast<uint64_t>(1) << SUDIV;
  }

  if (!IsSupported(UNALIGNED_ACCESSES) && FLAG_enable_unaligned_accesses
      && OS::ArmCpuHasFeature(ARMv7)) {
    found_by_runtime_probing_only_ |=
        static_cast<uint64_t>(1) << UNALIGNED_ACCESSES;
  }

  CpuImplementer implementer = OS::GetCpuImplementer();
  if (implementer == QUALCOMM_IMPLEMENTER &&
      FLAG_enable_movw_movt && OS::ArmCpuHasFeature(ARMv7)) {
    found_by_runtime_probing_only_ |=
        static_cast<uint64_t>(1) << MOVW_MOVT_IMMEDIATE_LOADS;
  }

  CpuPart part = OS::GetCpuPart(implementer);
  if ((part == CORTEX_A9) || (part == CORTEX_A5)) {
    cache_line_size_ = 32;
  }

  if (!IsSupported(VFP32DREGS) && FLAG_enable_32dregs
      && OS::ArmCpuHasFeature(VFP32DREGS)) {
    found_by_runtime_probing_only_ |= static_cast<uint64_t>(1) << VFP32DREGS;
  }

  supported_ |= found_by_runtime_probing_only_;
#endif

  // Assert that VFP3 implies ARMv7.
  ASSERT(!IsSupported(VFP3) || IsSupported(ARMv7));
}


void CpuFeatures::PrintTarget() {
  const char* arm_arch = NULL;
  const char* arm_test = "";
  const char* arm_fpu = "";
  const char* arm_thumb = "";
  const char* arm_float_abi = NULL;

#if defined CAN_USE_ARMV7_INSTRUCTIONS
  arm_arch = "arm v7";
#else
  arm_arch = "arm v6";
#endif

#ifdef __arm__

# ifdef ARM_TEST
  arm_test = " test";
# endif
# if defined __ARM_NEON__
  arm_fpu = " neon";
# elif defined CAN_USE_VFP3_INSTRUCTIONS
  arm_fpu = " vfp3";
# else
  arm_fpu = " vfp2";
# endif
# if (defined __thumb__) || (defined __thumb2__)
  arm_thumb = " thumb";
# endif
  arm_float_abi = OS::ArmUsingHardFloat() ? "hard" : "softfp";

#else  // __arm__

  arm_test = " simulator";
# if defined CAN_USE_VFP3_INSTRUCTIONS
#  if defined CAN_USE_VFP32DREGS
  arm_fpu = " vfp3";
#  else
  arm_fpu = " vfp3-d16";
#  endif
# else
  arm_fpu = " vfp2";
# endif
# if USE_EABI_HARDFLOAT == 1
  arm_float_abi = "hard";
# else
  arm_float_abi = "softfp";
# endif

#endif  // __arm__

  printf("target%s %s%s%s %s\n",
         arm_test, arm_arch, arm_fpu, arm_thumb, arm_float_abi);
}


void CpuFeatures::PrintFeatures() {
  printf(
    "ARMv7=%d VFP3=%d VFP32DREGS=%d NEON=%d SUDIV=%d UNALIGNED_ACCESSES=%d "
    "MOVW_MOVT_IMMEDIATE_LOADS=%d",
    CpuFeatures::IsSupported(ARMv7),
    CpuFeatures::IsSupported(VFP3),
    CpuFeatures::IsSupported(VFP32DREGS),
    CpuFeatures::IsSupported(NEON),
    CpuFeatures::IsSupported(SUDIV),
    CpuFeatures::IsSupported(UNALIGNED_ACCESSES),
    CpuFeatures::IsSupported(MOVW_MOVT_IMMEDIATE_LOADS));
#ifdef __arm__
  bool eabi_hardfloat = OS::ArmUsingHardFloat();
#elif USE_EABI_HARDFLOAT
  bool eabi_hardfloat = true;
#else
  bool eabi_hardfloat = false;
#endif
    printf(" USE_EABI_HARDFLOAT=%d\n", eabi_hardfloat);
}


// -----------------------------------------------------------------------------
// Implementation of RelocInfo

const int RelocInfo::kApplyMask = 0;


bool RelocInfo::IsCodedSpecially() {
  // The deserializer needs to know whether a pointer is specially coded.  Being
  // specially coded on ARM means that it is a movw/movt instruction.  We don't
  // generate those yet.
  return false;
}


void RelocInfo::PatchCode(byte* instructions, int instruction_count) {
  // Patch the code at the current address with the supplied instructions.
#ifndef USE_THUMB
  Instr* pc = reinterpret_cast<Instr*>(pc_);
  Instr* instr = reinterpret_cast<Instr*>(instructions);
#else
  Instr16* pc = reinterpret_cast<Instr16*>(pc_);
  Instr16* instr = reinterpret_cast<Instr16*>(instructions);
#endif

  for (int i = 0; i < instruction_count; i++) {
    *(pc + i) = *(instr + i);
  }

  // Indicate that code has changed.
  CPU::FlushICache(pc_, instruction_count * Assembler::kInstrSize);
}


// Patch the code at the current PC with a call to the target address.
// Additional guard instructions can be added if required.
void RelocInfo::PatchCodeWithCall(Address target, int guard_bytes) {
  // Patch the code at the current address with a call to the target.
  UNIMPLEMENTED();
}


// -----------------------------------------------------------------------------
// Implementation of Operand and MemOperand
// See assembler-arm-inl.h for inlined constructors

Operand::Operand(Handle<Object> handle) {
#ifdef DEBUG
  Isolate* isolate = Isolate::Current();
#endif
  AllowDeferredHandleDereference using_raw_address;
  rm_ = no_reg;
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  ASSERT(!isolate->heap()->InNewSpace(obj));
  if (obj->IsHeapObject()) {
    imm32_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // no relocation needed
    imm32_ = reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE32;
  }
}


Operand::Operand(Register rm, ShiftOp shift_op, int shift_imm) {
  ASSERT(is_uint5(shift_imm));
  rm_ = rm;
  rs_ = no_reg;
  shift_op_ = shift_op;
  shift_imm_ = shift_imm & 31;
  if (shift_op == RRX) {
    // encoded as ROR with shift_imm == 0
    ASSERT(shift_imm == 0);
    shift_op_ = ROR;
    shift_imm_ = 0;
  }
}


Operand::Operand(Register rm, ShiftOp shift_op, Register rs) {
  ASSERT(shift_op != RRX);
  rm_ = rm;
  rs_ = no_reg;
  shift_op_ = shift_op;
  rs_ = rs;
}


MemOperand::MemOperand(Register rn, int32_t offset, AddrMode am) {
  rn_ = rn;
  rm_ = no_reg;
  offset_ = offset;
  am_ = am;
}


MemOperand::MemOperand(Register rn, Register rm, AddrMode am) {
  rn_ = rn;
  rm_ = rm;
  shift_op_ = LSL;
  shift_imm_ = 0;
  am_ = am;
}


MemOperand::MemOperand(Register rn, Register rm,
                       ShiftOp shift_op, int shift_imm, AddrMode am) {
  ASSERT(is_uint5(shift_imm));
  rn_ = rn;
  rm_ = rm;
  shift_op_ = shift_op;
  shift_imm_ = shift_imm & 31;
  am_ = am;
}


NeonMemOperand::NeonMemOperand(Register rn, AddrMode am, int align) {
  ASSERT((am == Offset) || (am == PostIndex));
  rn_ = rn;
  rm_ = (am == Offset) ? pc : sp;
  SetAlignment(align);
}


NeonMemOperand::NeonMemOperand(Register rn, Register rm, int align) {
  rn_ = rn;
  rm_ = rm;
  SetAlignment(align);
}


void NeonMemOperand::SetAlignment(int align) {
  switch (align) {
    case 0:
      align_ = 0;
      break;
    case 64:
      align_ = 1;
      break;
    case 128:
      align_ = 2;
      break;
    case 256:
      align_ = 3;
      break;
    default:
      UNREACHABLE();
      align_ = 0;
      break;
  }
}


NeonListOperand::NeonListOperand(DoubleRegister base, int registers_count) {
  base_ = base;
  switch (registers_count) {
    case 1:
      type_ = nlt_1;
      break;
    case 2:
      type_ = nlt_2;
      break;
    case 3:
      type_ = nlt_3;
      break;
    case 4:
      type_ = nlt_4;
      break;
    default:
      UNREACHABLE();
      type_ = nlt_1;
      break;
  }
}



Assembler::Assembler(Isolate* isolate, void* buffer, int buffer_size)
    : AssemblerBase(isolate, buffer, buffer_size),
      recorded_ast_id_(TypeFeedbackId::None()),
      positions_recorder_(this) {
  reloc_info_writer.Reposition(buffer_ + buffer_size_, pc_);
  num_pending_reloc_info_ = 0;
  num_pending_64_bit_reloc_info_ = 0;
#ifdef USE_THUMB
  it_instr_pos_ = -1;
  it_size_ = 0;
#endif
  next_buffer_check_ = 0;
  const_pool_blocked_nesting_ = 0;
  no_const_pool_before_ = 0;
  first_const_pool_use_ = -1;
  last_bound_pos_ = 0;
  ClearRecordedAstId();
  ResetKilled();
}


Assembler::~Assembler() {
  ASSERT(const_pool_blocked_nesting_ == 0);
}


void Assembler::GetCode(CodeDesc* desc) {
  // Emit constant pool if necessary.
  CheckConstPool(true, false);
  ASSERT(num_pending_reloc_info_ == 0);
  ASSERT(num_pending_64_bit_reloc_info_ == 0);

  // Set up code descriptor.
  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
}


void Assembler::Align(int m) {
  ASSERT(m >= 4 && IsPowerOf2(m));
  while ((pc_offset() & (m - 1)) != 0) {
    nop();
  }
}


void Assembler::CodeTargetAlign() {
  // Preferred alignment of jump targets on some ARM chips.
  Align(8);
}

// Labels refer to positions in the (to be) generated code.
// There are bound, linked, and unused labels.
//
// Bound labels refer to known positions in the already
// generated code. pos() is the position the label refers to.
//
// Linked labels refer to unknown positions in the code
// to be generated; pos() is the position of the last
// instruction using the label.


// The link chain is terminated by a negative code position (must be aligned)
const int kEndOfChain = -4;

void Assembler::bind_to(Label* L, int pos) {
  ASSERT(0 <= pos && pos <= pc_offset());  // must have a valid binding position
  while (L->is_linked()) {
    int fixup_pos = L->pos();
    next(L);  // call next before overwriting link with target at fixup_pos
    target_at_put(fixup_pos, pos);
  }
  L->bind_to(pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pos > last_bound_pos_)
    last_bound_pos_ = pos;
}


void Assembler::link_to(Label* L, Label* appendix) {
  if (appendix->is_linked()) {
    if (L->is_linked()) {
      // Append appendix to L's list.
      int fixup_pos;
      int link = L->pos();
      do {
        fixup_pos = link;
        link = target_at(fixup_pos);
      } while (link > 0);
      ASSERT(link == kEndOfChain);
      target_at_put(fixup_pos, appendix->pos());
    } else {
      // L is empty, simply use appendix.
      *L = *appendix;
    }
  }
  appendix->Unuse();  // appendix should not be used anymore
}


void Assembler::bind(Label* L) {
  ASSERT(!L->is_bound());  // label can only be bound once
#ifdef USE_THUMB
  flush_cond();
#endif
  bind_to(L, pc_offset());
}


void Assembler::next(Label* L) {
  ASSERT(L->is_linked());
  int link = target_at(L->pos());
  if (link == kEndOfChain) {
    L->Unuse();
  } else {
    ASSERT(link >= 0);
    L->link_to(link);
  }
}

// We have to use the temporary register for things that can be relocated even
// if they can be encoded in the ARM's 12 bits of immediate-offset instruction
// space.  There is no guarantee that the relocated location can be similarly
// encoded.
bool Operand::must_output_reloc_info(const Assembler* assembler) const {
  if (rmode_ == RelocInfo::EXTERNAL_REFERENCE) {
#ifdef DEBUG
    if (!Serializer::enabled()) {
      Serializer::TooLateToEnableNow();
    }
#endif  // def DEBUG
    if (assembler != NULL && assembler->predictable_code_size()) return true;
    return Serializer::enabled();
  } else if (RelocInfo::IsNone(rmode_)) {
    return false;
  }
  return true;
}

int Assembler::branch_offset(Label* L, bool jump_elimination_allowed) {
  int target_pos;
  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link
    } else {
      target_pos = kEndOfChain;
    }
    L->link_to(pc_offset());
  }

  // Block the emission of the constant pool, since the branch instruction must
  // be emitted at the pc offset recorded by the label.
  BlockConstPoolFor(1);
  return target_pos - (pc_offset() + kPcLoadDelta);
}


void Assembler::label_at_put(Label* L, int at_offset) {
  int target_pos;
  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link
    } else {
      target_pos = kEndOfChain;
    }
    L->link_to(at_offset);
#ifndef USE_THUMB
    instr_at_put(at_offset, target_pos + (Code::kHeaderSize - kHeapObjectTag));
#else
    instr32_at_put(at_offset, target_pos + (Code::kHeaderSize - kHeapObjectTag));
#endif
  }
}

// Exception-generating instructions and debugging support.
// Stops with a non-negative code less than kNumOfWatchedStops support
// enabling/disabling and a counter feature. See simulator-arm.h .
void Assembler::stop(const char* msg, Condition cond, int32_t code) {
#ifndef __arm__
  ASSERT(code >= kDefaultStopCode);
  {
    // The Simulator will handle the stop instruction and get the message
    // address. It expects to find the address just after the svc instruction.
    BlockConstPoolScope block_const_pool(this);
    if (code >= 0) {
      svc(kStopCode + code, cond);
    } else {
      svc(kStopCode + kMaxStopCode, cond);
    }
    emit(reinterpret_cast<Instr>(msg));
  }
#else  // def __arm__
  if (cond != al) {
    Label skip;
    b(&skip, NegateCondition(cond));
    bkpt(0);
    bind(&skip);
  } else {
    bkpt(0);
  }
#endif  // def __arm__
}

// Debugging.
void Assembler::RecordJSReturn() {
  positions_recorder()->WriteRecordedPositions();
  CheckBuffer();
  RecordRelocInfo(RelocInfo::JS_RETURN);
}


void Assembler::RecordDebugBreakSlot() {
  positions_recorder()->WriteRecordedPositions();
  CheckBuffer();
  RecordRelocInfo(RelocInfo::DEBUG_BREAK_SLOT);
}


void Assembler::RecordComment(const char* msg) {
  if (FLAG_code_comments) {
    CheckBuffer();
    RecordRelocInfo(RelocInfo::COMMENT, reinterpret_cast<intptr_t>(msg));
  }
}


void Assembler::RecordConstPool(int size) {
  // We only need this for debugger support, to correctly compute offsets in the
  // code.
#ifdef ENABLE_DEBUGGER_SUPPORT
  RecordRelocInfo(RelocInfo::CONST_POOL, static_cast<intptr_t>(size));
#endif
}


void Assembler::GrowBuffer() {
  if (!own_buffer_) FATAL("external code buffer is too small");

  // Compute new buffer size.
  CodeDesc desc;  // the new buffer
  if (buffer_size_ < 4*KB) {
    desc.buffer_size = 4*KB;
  } else if (buffer_size_ < 1*MB) {
    desc.buffer_size = 2*buffer_size_;
  } else {
    desc.buffer_size = buffer_size_ + 1*MB;
  }
  CHECK_GT(desc.buffer_size, 0);  // no overflow

  // Set up new buffer.
  desc.buffer = NewArray<byte>(desc.buffer_size);

  desc.instr_size = pc_offset();
  desc.reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();

  // Copy the data.
  int pc_delta = desc.buffer - buffer_;
  int rc_delta = (desc.buffer + desc.buffer_size) - (buffer_ + buffer_size_);
  OS::MemMove(desc.buffer, buffer_, desc.instr_size);
  OS::MemMove(reloc_info_writer.pos() + rc_delta,
              reloc_info_writer.pos(), desc.reloc_size);

  // Switch buffers.
  DeleteArray(buffer_);
  buffer_ = desc.buffer;
  buffer_size_ = desc.buffer_size;
  pc_ += pc_delta;
  reloc_info_writer.Reposition(reloc_info_writer.pos() + rc_delta,
                               reloc_info_writer.last_pc() + pc_delta);

  // None of our relocation types are pc relative pointing outside the code
  // buffer nor pc absolute pointing inside the code buffer, so there is no need
  // to relocate any emitted relocation entries.

  // Relocate pending relocation entries.
  for (int i = 0; i < num_pending_reloc_info_; i++) {
    RelocInfo& rinfo = pending_reloc_info_[i];
    ASSERT(rinfo.rmode() != RelocInfo::COMMENT &&
           rinfo.rmode() != RelocInfo::POSITION);
    if (rinfo.rmode() != RelocInfo::JS_RETURN) {
      rinfo.set_pc(rinfo.pc() + pc_delta);
    }
  }
}


void Assembler::db(uint8_t data) {
  // No relocation info should be pending while using db. db is used
  // to write pure data with no pointers and the constant pool should
  // be emitted before using db.
  ASSERT(num_pending_reloc_info_ == 0);
  ASSERT(num_pending_64_bit_reloc_info_ == 0);
  CheckBuffer();
  *reinterpret_cast<uint8_t*>(pc_) = data;
  pc_ += sizeof(uint8_t);
}


void Assembler::dd(uint32_t data) {
  // No relocation info should be pending while using dd. dd is used
  // to write pure data with no pointers and the constant pool should
  // be emitted before using dd.
  ASSERT(num_pending_reloc_info_ == 0);
  ASSERT(num_pending_64_bit_reloc_info_ == 0);
  CheckBuffer();
  *reinterpret_cast<uint32_t*>(pc_) = data;
  pc_ += sizeof(uint32_t);
}


void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data,
                                UseConstantPoolMode mode) {
  // We do not try to reuse pool constants.
  RelocInfo rinfo(pc_, rmode, data, NULL);
  if (((rmode >= RelocInfo::JS_RETURN) &&
       (rmode <= RelocInfo::DEBUG_BREAK_SLOT)) ||
      (rmode == RelocInfo::CONST_POOL) ||
      mode == DONT_USE_CONSTANT_POOL) {
    // Adjust code for new modes.
    ASSERT(RelocInfo::IsDebugBreakSlot(rmode)
           || RelocInfo::IsJSReturn(rmode)
           || RelocInfo::IsComment(rmode)
           || RelocInfo::IsPosition(rmode)
           || RelocInfo::IsConstPool(rmode)
           || mode == DONT_USE_CONSTANT_POOL);
    // These modes do not need an entry in the constant pool.
  } else {
    RecordRelocInfoConstantPoolEntryHelper(rinfo);
  }
  if (!RelocInfo::IsNone(rinfo.rmode())) {
    // Don't record external references unless the heap will be serialized.
    if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
#ifdef DEBUG
      if (!Serializer::enabled()) {
        Serializer::TooLateToEnableNow();
      }
#endif
      if (!Serializer::enabled() && !emit_debug_code()) {
        return;
      }
    }
    ASSERT(buffer_space() >= kMaxRelocSize);  // too late to grow buffer here
    if (rmode == RelocInfo::CODE_TARGET_WITH_ID) {
      RelocInfo reloc_info_with_ast_id(pc_,
                                       rmode,
                                       RecordedAstId().ToInt(),
                                       NULL);
      ClearRecordedAstId();
      reloc_info_writer.Write(&reloc_info_with_ast_id);
    } else {
      reloc_info_writer.Write(&rinfo);
    }
  }
}


void Assembler::RecordRelocInfo(double data) {
  // We do not try to reuse pool constants.
  RelocInfo rinfo(pc_, data);
  RecordRelocInfoConstantPoolEntryHelper(rinfo);
}


void Assembler::RecordRelocInfoConstantPoolEntryHelper(const RelocInfo& rinfo) {
  ASSERT(num_pending_reloc_info_ < kMaxNumPendingRelocInfo);
  if (num_pending_reloc_info_ == 0) {
    first_const_pool_use_ = pc_offset();
  }
  pending_reloc_info_[num_pending_reloc_info_++] = rinfo;
  if (rinfo.rmode() == RelocInfo::NONE64) {
    ++num_pending_64_bit_reloc_info_;
  }
  ASSERT(num_pending_64_bit_reloc_info_ <= num_pending_reloc_info_);
  // Make sure the constant pool is not emitted in place of the next
  // instruction for which we just recorded relocation info.
  BlockConstPoolFor(1);
}


void Assembler::BlockConstPoolFor(int instructions) {
  int pc_limit = pc_offset() + instructions * kInstrSize;
  if (no_const_pool_before_ < pc_limit) {
    // If there are some pending entries, the constant pool cannot be blocked
    // further than constant pool instruction's reach.
    ASSERT((num_pending_reloc_info_ == 0) ||
           (pc_limit - first_const_pool_use_ < kMaxDistToIntPool));
    // TODO(jfb) Also check 64-bit entries are in range (requires splitting
    //           them up from 32-bit entries).
    no_const_pool_before_ = pc_limit;
  }

  if (next_buffer_check_ < no_const_pool_before_) {
    next_buffer_check_ = no_const_pool_before_;
  }
}


void Assembler::CheckConstPool(bool force_emit, bool require_jump) {
  // Some short sequence of instruction mustn't be broken up by constant pool
  // emission, such sequences are protected by calls to BlockConstPoolFor and
  // BlockConstPoolScope.
  if (is_const_pool_blocked()) {
    // Something is wrong if emission is forced and blocked at the same time.
    ASSERT(!force_emit);
    return;
  }

  // There is nothing to do if there are no pending constant pool entries.
  if (num_pending_reloc_info_ == 0)  {
    ASSERT(num_pending_64_bit_reloc_info_ == 0);
    // Calculate the offset of the next check.
    next_buffer_check_ = pc_offset() + kCheckPoolInterval;
    return;
  }

  // Check that the code buffer is large enough before emitting the constant
  // pool (include the jump over the pool and the constant pool marker and
  // the gap to the relocation information).
  // Note 64-bit values are wider, and the first one needs to be 64-bit aligned.
#ifndef USE_THUMB
  const int jump_size = kInstrSize;
  const int long_marker_size = kInstrSize;
#else
  // we'll use 32bit jumps for now to be predictable
  const int jump_size = 2*kInstrSize;
  const int long_marker_size = 2*kInstrSize;
  const int short_marker_size = 2*kInstrSize;
#endif

  int jump_instr = require_jump ? jump_size : 0;
  int size_up_to_marker = jump_instr + long_marker_size;
  int size_after_marker = num_pending_reloc_info_ * kPointerSize;
  bool has_fp_values = (num_pending_64_bit_reloc_info_ > 0);

#ifdef USE_THUMB
  // on thumb we need to take care to align to 32-bit
  bool require_32_bit_align =
      (((uintptr_t)pc_ + size_up_to_marker + size_after_marker) & 0x1);
  if (require_32_bit_align) {
    size_after_marker += short_marker_size;
  }
#endif

  // 64-bit values must be 64-bit aligned.
  // We'll start emitting at PC: branch+marker, then 32-bit values, then
  // 64-bit values which might need to be aligned.
  bool require_64_bit_align = has_fp_values &&
      (((uintptr_t)pc_ + size_up_to_marker + size_after_marker) & 0x3);
  if (require_64_bit_align) {
    size_after_marker += long_marker_size;
  }
  // num_pending_reloc_info_ also contains 64-bit entries, the above code
  // therefore already counted half of the size for 64-bit entries. Add the
  // remaining size.
  STATIC_ASSERT(kPointerSize == kDoubleSize / 2);
  size_after_marker += num_pending_64_bit_reloc_info_ * (kDoubleSize / 2);

  int size = size_up_to_marker + size_after_marker;

  // We emit a constant pool when:
  //  * requested to do so by parameter force_emit (e.g. after each function).
  //  * the distance from the first instruction accessing the constant pool to
  //    any of the constant pool entries will exceed its limit the next
  //    time the pool is checked. This is overly restrictive, but we don't emit
  //    constant pool entries in-order so it's conservatively correct.
  //  * the instruction doesn't require a jump after itself to jump over the
  //    constant pool, and we're getting close to running out of range.
  if (!force_emit) {
    ASSERT((first_const_pool_use_ >= 0) && (num_pending_reloc_info_ > 0));
    int dist = pc_offset() + size - first_const_pool_use_;
    if (has_fp_values) {
      if ((dist < kMaxDistToFPPool - kCheckPoolInterval) &&
          (require_jump || (dist < kMaxDistToFPPool / 2))) {
        return;
      }
    } else {
      if ((dist < kMaxDistToIntPool - kCheckPoolInterval) &&
          (require_jump || (dist < kMaxDistToIntPool / 2))) {
        return;
      }
    }
  }

  int needed_space = size + kGap;
  while (buffer_space() <= needed_space) GrowBuffer();

  {
    // Block recursive calls to CheckConstPool.
    BlockConstPoolScope block_const_pool(this);
    RecordComment("[ Constant Pool");
    RecordConstPool(size);

    // Emit jump over constant pool if necessary.
    Label after_pool;
    if (require_jump) {
      b(&after_pool);
    }

    // Put down constant pool marker "Undefined instruction".
    // The data size helps disassembly know what to print.
#ifndef USE_THUMB
    emit(arm::kConstantPoolMarker | arm::EncodeConstantPoolLength(size_after_marker));
    if (require_64_bit_align) {
      emit(arm::kConstantPoolMarker);
    }
#else
    emit32(thumb32::kConstantPoolMarker | thumb32::EncodeConstantPoolLength(size_after_marker), kSpecialCondition);
    if (require_32_bit_align) {
      emit16(thumb16::kConstantPoolMarker, kSpecialCondition);
    }
    if (require_64_bit_align) {
      emit32(thumb32::kConstantPoolMarker, kSpecialCondition);
    }
#endif
    // Emit 64-bit constant pool entries first: their range is smaller than
    // 32-bit entries.
    for (int i = 0; i < num_pending_reloc_info_; i++) {
      RelocInfo& rinfo = pending_reloc_info_[i];

      if (rinfo.rmode() != RelocInfo::NONE64) {
        // 32-bit values emitted later.
        continue;
      }

      ASSERT(!((uintptr_t)pc_ & 0x3));  // Check 64-bit alignment.

      // Instruction to patch must be 'vldr rd, [pc, #offset]' with offset == 0.
#ifndef USE_THUMB
      Instr instr = instr_at(rinfo.pc());
      ASSERT((arm::IsVldrDPcImmediateOffset(instr) &&
              arm::GetVldrDRegisterImmediateOffset(instr) == 0));
#else
      Instr32 instr = instr32_at(rinfo.pc());
      ASSERT((thumb32::IsVldrDPcImmediateOffset(instr) &&
              thumb32::GetVldrDRegisterImmediateOffset(instr) == 0));
#endif
      int delta = pc_ - RoundDown(rinfo.pc(), 4) - kPcLoadDelta;

      ASSERT(is_uint10(delta));
#ifndef USE_THUMB
      instr_at_put(rinfo.pc(), arm::SetVldrDRegisterImmediateOffset(instr, delta));
#else
      instr32_at_put(rinfo.pc(), thumb32::SetVldrDRegisterImmediateOffset(instr, delta));
#endif

      const double double_data = rinfo.data64();
      uint64_t uint_data = 0;
      OS::MemCopy(&uint_data, &double_data, sizeof(double_data));
      emit(uint_data & 0xFFFFFFFF);
      emit(uint_data >> 32);
    }

    // Emit 32-bit constant pool entries.
    for (int i = 0; i < num_pending_reloc_info_; i++) {
      RelocInfo& rinfo = pending_reloc_info_[i];
      ASSERT(rinfo.rmode() != RelocInfo::COMMENT &&
             rinfo.rmode() != RelocInfo::POSITION &&
             rinfo.rmode() != RelocInfo::STATEMENT_POSITION &&
             rinfo.rmode() != RelocInfo::CONST_POOL);

      if (rinfo.rmode() == RelocInfo::NONE64) {
        // 64-bit values emitted earlier.
        continue;
      }

#ifndef USE_THUMB
      Instr instr = instr_at(rinfo.pc());
      // 64-bit loads shouldn't get here.
      ASSERT(!arm::IsVldrDPcImmediateOffset(instr));
#else
      Instr32 instr = instr32_at(rinfo.pc());
      ASSERT(!thumb32::IsVldrDPcImmediateOffset(instr));
#endif
      int delta = pc_ - RoundDown(rinfo.pc(), 4) - kPcLoadDelta;
      // 0 is the smallest delta:
      //   ldr rd, [pc, #0]
      //   constant pool marker
      //   data
#ifndef USE_THUMB
      if (arm::IsLdrPcImmediateOffset(instr) &&
          arm::GetLdrRegisterImmediateOffset(instr) == 0) {
        ASSERT(is_uint12(delta));
        instr_at_put(rinfo.pc(), arm::SetLdrRegisterImmediateOffset(instr, delta));
        emit(rinfo.data());
      } else {
        ASSERT(arm::IsMovW(instr));
        emit(rinfo.data());
      }
#else
      if (thumb32::IsLdrPcImmediateOffset(instr) &&
          thumb32::GetLdrRegisterImmediateOffset(instr) == 0) {
        ASSERT(is_uint12(delta));
        instr32_at_put(rinfo.pc(), thumb32::SetLdrRegisterImmediateOffset(instr, delta));
        emit(rinfo.data());
      } else {
        ASSERT(thumb32::IsMovW(instr));
        emit(rinfo.data());
      }
#endif
    }

    num_pending_reloc_info_ = 0;
    num_pending_64_bit_reloc_info_ = 0;
    first_const_pool_use_ = -1;

    RecordComment("]");

    if (after_pool.is_linked()) {
      bind(&after_pool);
    }
  }

  // Since a constant pool was just emitted, move the check offset forward by
  // the standard interval.
  next_buffer_check_ = pc_offset() + kCheckPoolInterval;
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
