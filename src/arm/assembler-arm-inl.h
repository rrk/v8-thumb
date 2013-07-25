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

#ifndef V8_ARM_ASSEMBLER_ARM_INL_H_
#define V8_ARM_ASSEMBLER_ARM_INL_H_

#include "arm/assembler-arm.h"

#include "cpu.h"
#include "debug.h"


namespace v8 {
namespace internal {


int Register::NumAllocatableRegisters() {
  return kMaxNumAllocatableRegisters;
}

bool Register::is_pcsp() const {
  return is(pc) || is(sp);
}

int DwVfpRegister::NumRegisters() {
  return CpuFeatures::IsSupported(VFP32DREGS) ? 32 : 16;
}


int DwVfpRegister::NumAllocatableRegisters() {
  return NumRegisters() - kNumReservedRegisters;
}


int DwVfpRegister::ToAllocationIndex(DwVfpRegister reg) {
  ASSERT(!reg.is(kDoubleRegZero));
  ASSERT(!reg.is(kScratchDoubleReg));
  if (reg.code() > kDoubleRegZero.code()) {
    return reg.code() - kNumReservedRegisters;
  }
  return reg.code();
}


DwVfpRegister DwVfpRegister::FromAllocationIndex(int index) {
  ASSERT(index >= 0 && index < NumAllocatableRegisters());
  ASSERT(kScratchDoubleReg.code() - kDoubleRegZero.code() ==
         kNumReservedRegisters - 1);
  if (index >= kDoubleRegZero.code()) {
    return from_code(index + kNumReservedRegisters);
  }
  return from_code(index);
}


void RelocInfo::apply(intptr_t delta) {
  if (RelocInfo::IsInternalReference(rmode_)) {
    // absolute code pointer inside code object moves with the code object.
    int32_t* p = reinterpret_cast<int32_t*>(pc_);
    *p += delta;  // relocate entry
  }
  // We do not use pc relative addressing on ARM, so there is
  // nothing else to do.
}

inline Address target_address_or_pointer_at(byte *pc, RelocInfo::Mode rmode) {
#ifdef USE_THUMB
  if (RelocInfo::IsCodeTarget(rmode) || RelocInfo::IsRuntimeEntry(rmode)) {
    return Assembler::target_address_at(pc);
  } else {
    return Assembler::target_pointer_at(pc);
  }
#else
  return Assembler::target_pointer_at(pc);
#endif
}

inline void set_target_address_or_pointer_at(byte *pc, Address target, RelocInfo::Mode rmode) {
#ifdef USE_THUMB
  if (RelocInfo::IsCodeTarget(rmode) || RelocInfo::IsRuntimeEntry(rmode)) {
    Assembler::set_target_address_at(pc, target);
  } else {
    Assembler::set_target_pointer_at(pc, target);
  }
#else
  Assembler::set_target_pointer_at(pc, target);
#endif
}


Address RelocInfo::target_address() {
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_));
  return Assembler::target_address_at(pc_);
}


Address RelocInfo::target_address_address() {
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_)
                              || rmode_ == EMBEDDED_OBJECT
                              || rmode_ == EXTERNAL_REFERENCE);
  return reinterpret_cast<Address>(Assembler::target_pointer_address_at(pc_));
}


int RelocInfo::target_address_size() {
  return kPointerSize;
}


void RelocInfo::set_target_address(Address target, WriteBarrierMode mode) {
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_));
  Assembler::set_target_address_at(pc_, target);
  if (mode == UPDATE_WRITE_BARRIER && host() != NULL && IsCodeTarget(rmode_)) {
    Object* target_code = Code::GetCodeFromTargetAddress(target);
    host()->GetHeap()->incremental_marking()->RecordWriteIntoCode(
        host(), this, HeapObject::cast(target_code));
  }
}


Object* RelocInfo::target_object() {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return reinterpret_cast<Object*>(target_address_or_pointer_at(pc_, rmode_));
}


Handle<Object> RelocInfo::target_object_handle(Assembler* origin) {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  Address p = target_address_or_pointer_at(pc_, rmode_);
  return Handle<Object>(reinterpret_cast<Object**>(p));
}


Object** RelocInfo::target_object_address() {
  // Provide a "natural pointer" to the embedded object,
  // which can be de-referenced during heap iteration.
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  Address p = target_address_or_pointer_at(pc_, rmode_);
  reconstructed_obj_ptr_ = reinterpret_cast<Object*>(p);
  return &reconstructed_obj_ptr_;
}


void RelocInfo::set_target_object(Object* target, WriteBarrierMode mode) {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  ASSERT(!target->IsConsString());
  Address p = reinterpret_cast<Address>(target);
  set_target_address_or_pointer_at(pc_, p, rmode_);
  if (mode == UPDATE_WRITE_BARRIER &&
      host() != NULL &&
      target->IsHeapObject()) {
    host()->GetHeap()->incremental_marking()->RecordWrite(
        host(), &Memory::Object_at(pc_), HeapObject::cast(target));
  }
}


Address* RelocInfo::target_reference_address() {
  ASSERT(rmode_ == EXTERNAL_REFERENCE);
  reconstructed_adr_ptr_ = Assembler::target_pointer_at(pc_);
  return &reconstructed_adr_ptr_;
}


Address RelocInfo::target_runtime_entry(Assembler* origin) {
  ASSERT(IsRuntimeEntry(rmode_));
  return target_address();
}


void RelocInfo::set_target_runtime_entry(Address target,
                                         WriteBarrierMode mode) {
  ASSERT(IsRuntimeEntry(rmode_));
  if (target_address() != target) set_target_address(target, mode);
}


Handle<Cell> RelocInfo::target_cell_handle() {
  ASSERT(rmode_ == RelocInfo::CELL);
  Address address = Memory::Address_at(pc_);
  return Handle<Cell>(reinterpret_cast<Cell**>(address));
}


Cell* RelocInfo::target_cell() {
  ASSERT(rmode_ == RelocInfo::CELL);
  return Cell::FromValueAddress(Memory::Address_at(pc_));
}


void RelocInfo::set_target_cell(Cell* cell, WriteBarrierMode mode) {
  ASSERT(rmode_ == RelocInfo::CELL);
  Address address = cell->address() + Cell::kValueOffset;
  Memory::Address_at(pc_) = address;
  if (mode == UPDATE_WRITE_BARRIER && host() != NULL) {
    // TODO(1550) We are passing NULL as a slot because cell can never be on
    // evacuation candidate.
    host()->GetHeap()->incremental_marking()->RecordWrite(
        host(), NULL, cell);
  }
}

#ifndef USE_THUMB
static const int kNoCodeAgeSequenceLength = 3;
#else
static const int kNoCodeAgeSequenceLength = 6;
#endif

Code* RelocInfo::code_age_stub() {
  ASSERT(rmode_ == RelocInfo::CODE_AGE_SEQUENCE);
  Address addr = Memory::Address_at(pc_ + Assembler::kInstrSize * kNoCodeAgeSequenceLength - kPointerSize);
  return Code::GetCodeFromTargetAddress(CPU::DecodePcAddress(addr));
}


void RelocInfo::set_code_age_stub(Code* stub) {
  ASSERT(rmode_ == RelocInfo::CODE_AGE_SEQUENCE);
  Address addr = CPU::EncodePcAddress(stub->instruction_start());
  Memory::Address_at(pc_ + Assembler::kInstrSize * kNoCodeAgeSequenceLength - kPointerSize) = addr;
}


Address RelocInfo::call_address() {
  ASSERT((IsJSReturn(rmode()) && IsPatchedReturnSequence()) ||
         (IsDebugBreakSlot(rmode()) && IsPatchedDebugBreakSlotSequence()));
  return CPU::DecodePcAddress(Memory::Address_at(pc_ + Assembler::kDebugBreakSlotAddressOffset));

}


void RelocInfo::set_call_address(Address target) {
  ASSERT((IsJSReturn(rmode()) && IsPatchedReturnSequence()) ||
         (IsDebugBreakSlot(rmode()) && IsPatchedDebugBreakSlotSequence()));
  Memory::Address_at(pc_ + Assembler::kDebugBreakSlotAddressOffset) = CPU::EncodePcAddress(target);
  if (host() != NULL) {
    Object* target_code = Code::GetCodeFromTargetAddress(target);
    host()->GetHeap()->incremental_marking()->RecordWriteIntoCode(
        host(), this, HeapObject::cast(target_code));
  }
}


Object* RelocInfo::call_object() {
  return *call_object_address();
}


void RelocInfo::set_call_object(Object* target) {
  *call_object_address() = target;
}


Object** RelocInfo::call_object_address() {
  ASSERT((IsJSReturn(rmode()) && IsPatchedReturnSequence()) ||
         (IsDebugBreakSlot(rmode()) && IsPatchedDebugBreakSlotSequence()));
  return reinterpret_cast<Object**>(pc_ + Assembler::kDebugBreakSlotAddressOffset);
}



bool RelocInfo::IsPatchedDebugBreakSlotSequence() {
#ifndef USE_THUMB
  Instr current_instr = Assembler::instr_at(pc_);
  return !arm::IsNop(current_instr, Assembler::DEBUG_BREAK_NOP);
#else
  Instr16 current_instr = Assembler::instr16_at(pc_);
  return !thumb16::IsNop(current_instr, Assembler::DEBUG_BREAK_NOP);
#endif
}

void RelocInfo::Visit(ObjectVisitor* visitor) {
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    visitor->VisitEmbeddedPointer(this);
  } else if (RelocInfo::IsCodeTarget(mode)) {
    visitor->VisitCodeTarget(this);
  } else if (mode == RelocInfo::CELL) {
    visitor->VisitCell(this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    visitor->VisitExternalReference(this);
  } else if (RelocInfo::IsCodeAgeSequence(mode)) {
    visitor->VisitCodeAgeSequence(this);
#ifdef ENABLE_DEBUGGER_SUPPORT
  // TODO(isolates): Get a cached isolate below.
  } else if (((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence())) &&
             Isolate::Current()->debug()->has_break_points()) {
    visitor->VisitDebugTarget(this);
#endif
  } else if (RelocInfo::IsRuntimeEntry(mode)) {
    visitor->VisitRuntimeEntry(this);
  }
}


template<typename StaticVisitor>
void RelocInfo::Visit(Heap* heap) {
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    StaticVisitor::VisitEmbeddedPointer(heap, this);
  } else if (RelocInfo::IsCodeTarget(mode)) {
    StaticVisitor::VisitCodeTarget(heap, this);
  } else if (mode == RelocInfo::CELL) {
    StaticVisitor::VisitCell(heap, this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    StaticVisitor::VisitExternalReference(this);
  } else if (RelocInfo::IsCodeAgeSequence(mode)) {
    StaticVisitor::VisitCodeAgeSequence(heap, this);
#ifdef ENABLE_DEBUGGER_SUPPORT
  } else if (heap->isolate()->debug()->has_break_points() &&
             ((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence()))) {
    StaticVisitor::VisitDebugTarget(heap, this);
#endif
  } else if (RelocInfo::IsRuntimeEntry(mode)) {
    StaticVisitor::VisitRuntimeEntry(this);
  }
}


Operand::Operand(int32_t immediate, RelocInfo::Mode rmode)  {
  rm_ = no_reg;
  imm32_ = immediate;
  rmode_ = rmode;
}


Operand::Operand(const ExternalReference& f)  {
  rm_ = no_reg;
  imm32_ = reinterpret_cast<int32_t>(f.address());
  rmode_ = RelocInfo::EXTERNAL_REFERENCE;
}


Operand::Operand(Smi* value) {
  rm_ = no_reg;
  imm32_ =  reinterpret_cast<intptr_t>(value);
  rmode_ = RelocInfo::NONE32;
}


Operand::Operand(Register rm) {
  rm_ = rm;
  rs_ = no_reg;
  shift_op_ = LSL;
  shift_imm_ = 0;
}


bool Operand::is_reg() const {
  return rm_.is_valid() &&
         rs_.is(no_reg) &&
         shift_op_ == LSL &&
         shift_imm_ == 0;
}


void Assembler::CheckBuffer() {
  if (buffer_space() <= kGap) {
    GrowBuffer();
  }
  if (pc_offset() >= next_buffer_check_) {
    CheckConstPool(false, true);
  }
}


void Assembler::emit(Instr x) {
  CheckBuffer();
  *reinterpret_cast<Instr*>(pc_) = x;
  pc_ += sizeof(Instr);
}

void Assembler::deserialization_set_special_target_at(
    Address constant_pool_entry, Address target) {
  Memory::Address_at(constant_pool_entry) = target;
}


void Assembler::set_external_target_at(Address constant_pool_entry,
                                       Address target) {
  Memory::Address_at(constant_pool_entry) = target;
}

} }  // namespace v8::internal

#ifndef USE_THUMB
#include "arm/assembler-arm-arm-inl.h"
#else
#include "arm/assembler-arm-thumb-inl.h"
#endif

#endif  // V8_ARM_ASSEMBLER_ARM_INL_H_
