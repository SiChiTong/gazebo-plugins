// Generated by the protocol buffer compiler.  DO NOT EDIT!

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "MachineCommands.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace llsf_msgs {

namespace {

const ::google::protobuf::Descriptor* RemovePuckFromMachine_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  RemovePuckFromMachine_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* RemovePuckFromMachine_CompType_descriptor_ = NULL;
const ::google::protobuf::Descriptor* PlacePuckUnderMachine_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  PlacePuckUnderMachine_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* PlacePuckUnderMachine_CompType_descriptor_ = NULL;
const ::google::protobuf::Descriptor* LoadPuckInMachine_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  LoadPuckInMachine_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* LoadPuckInMachine_CompType_descriptor_ = NULL;

}  // namespace


void protobuf_AssignDesc_MachineCommands_2eproto() {
  protobuf_AddDesc_MachineCommands_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "MachineCommands.proto");
  GOOGLE_CHECK(file != NULL);
  RemovePuckFromMachine_descriptor_ = file->message_type(0);
  static const int RemovePuckFromMachine_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RemovePuckFromMachine, machine_name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RemovePuckFromMachine, puck_id_),
  };
  RemovePuckFromMachine_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      RemovePuckFromMachine_descriptor_,
      RemovePuckFromMachine::default_instance_,
      RemovePuckFromMachine_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RemovePuckFromMachine, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RemovePuckFromMachine, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(RemovePuckFromMachine));
  RemovePuckFromMachine_CompType_descriptor_ = RemovePuckFromMachine_descriptor_->enum_type(0);
  PlacePuckUnderMachine_descriptor_ = file->message_type(1);
  static const int PlacePuckUnderMachine_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PlacePuckUnderMachine, machine_name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PlacePuckUnderMachine, puck_id_),
  };
  PlacePuckUnderMachine_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      PlacePuckUnderMachine_descriptor_,
      PlacePuckUnderMachine::default_instance_,
      PlacePuckUnderMachine_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PlacePuckUnderMachine, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PlacePuckUnderMachine, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(PlacePuckUnderMachine));
  PlacePuckUnderMachine_CompType_descriptor_ = PlacePuckUnderMachine_descriptor_->enum_type(0);
  LoadPuckInMachine_descriptor_ = file->message_type(2);
  static const int LoadPuckInMachine_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LoadPuckInMachine, machine_name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LoadPuckInMachine, puck_id_),
  };
  LoadPuckInMachine_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      LoadPuckInMachine_descriptor_,
      LoadPuckInMachine::default_instance_,
      LoadPuckInMachine_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LoadPuckInMachine, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LoadPuckInMachine, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(LoadPuckInMachine));
  LoadPuckInMachine_CompType_descriptor_ = LoadPuckInMachine_descriptor_->enum_type(0);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_MachineCommands_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    RemovePuckFromMachine_descriptor_, &RemovePuckFromMachine::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    PlacePuckUnderMachine_descriptor_, &PlacePuckUnderMachine::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    LoadPuckInMachine_descriptor_, &LoadPuckInMachine::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_MachineCommands_2eproto() {
  delete RemovePuckFromMachine::default_instance_;
  delete RemovePuckFromMachine_reflection_;
  delete PlacePuckUnderMachine::default_instance_;
  delete PlacePuckUnderMachine_reflection_;
  delete LoadPuckInMachine::default_instance_;
  delete LoadPuckInMachine_reflection_;
}

void protobuf_AddDesc_MachineCommands_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\025MachineCommands.proto\022\tllsf_msgs\"f\n\025Re"
    "movePuckFromMachine\022\024\n\014machine_name\030\001 \002("
    "\t\022\017\n\007puck_id\030\002 \002(\r\"&\n\010CompType\022\014\n\007COMP_I"
    "D\020\320\017\022\014\n\010MSG_TYPE\020\016\"f\n\025PlacePuckUnderMach"
    "ine\022\024\n\014machine_name\030\001 \002(\t\022\017\n\007puck_id\030\002 \002"
    "(\r\"&\n\010CompType\022\014\n\007COMP_ID\020\320\017\022\014\n\010MSG_TYPE"
    "\020\017\"b\n\021LoadPuckInMachine\022\024\n\014machine_name\030"
    "\001 \002(\t\022\017\n\007puck_id\030\002 \002(\r\"&\n\010CompType\022\014\n\007CO"
    "MP_ID\020\320\017\022\014\n\010MSG_TYPE\020\020B8\n\037org.robocup_lo"
    "gistics.llsf_msgsB\025MachineCommandsProtos", 400);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "MachineCommands.proto", &protobuf_RegisterTypes);
  RemovePuckFromMachine::default_instance_ = new RemovePuckFromMachine();
  PlacePuckUnderMachine::default_instance_ = new PlacePuckUnderMachine();
  LoadPuckInMachine::default_instance_ = new LoadPuckInMachine();
  RemovePuckFromMachine::default_instance_->InitAsDefaultInstance();
  PlacePuckUnderMachine::default_instance_->InitAsDefaultInstance();
  LoadPuckInMachine::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_MachineCommands_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_MachineCommands_2eproto {
  StaticDescriptorInitializer_MachineCommands_2eproto() {
    protobuf_AddDesc_MachineCommands_2eproto();
  }
} static_descriptor_initializer_MachineCommands_2eproto_;


// ===================================================================

const ::google::protobuf::EnumDescriptor* RemovePuckFromMachine_CompType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return RemovePuckFromMachine_CompType_descriptor_;
}
bool RemovePuckFromMachine_CompType_IsValid(int value) {
  switch(value) {
    case 14:
    case 2000:
      return true;
    default:
      return false;
  }
}

#ifndef _MSC_VER
const RemovePuckFromMachine_CompType RemovePuckFromMachine::COMP_ID;
const RemovePuckFromMachine_CompType RemovePuckFromMachine::MSG_TYPE;
const RemovePuckFromMachine_CompType RemovePuckFromMachine::CompType_MIN;
const RemovePuckFromMachine_CompType RemovePuckFromMachine::CompType_MAX;
const int RemovePuckFromMachine::CompType_ARRAYSIZE;
#endif  // _MSC_VER
#ifndef _MSC_VER
const int RemovePuckFromMachine::kMachineNameFieldNumber;
const int RemovePuckFromMachine::kPuckIdFieldNumber;
#endif  // !_MSC_VER

RemovePuckFromMachine::RemovePuckFromMachine()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void RemovePuckFromMachine::InitAsDefaultInstance() {
}

RemovePuckFromMachine::RemovePuckFromMachine(const RemovePuckFromMachine& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void RemovePuckFromMachine::SharedCtor() {
  _cached_size_ = 0;
  machine_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  puck_id_ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

RemovePuckFromMachine::~RemovePuckFromMachine() {
  SharedDtor();
}

void RemovePuckFromMachine::SharedDtor() {
  if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
    delete machine_name_;
  }
  if (this != default_instance_) {
  }
}

void RemovePuckFromMachine::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* RemovePuckFromMachine::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return RemovePuckFromMachine_descriptor_;
}

const RemovePuckFromMachine& RemovePuckFromMachine::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_MachineCommands_2eproto();  return *default_instance_;
}

RemovePuckFromMachine* RemovePuckFromMachine::default_instance_ = NULL;

RemovePuckFromMachine* RemovePuckFromMachine::New() const {
  return new RemovePuckFromMachine;
}

void RemovePuckFromMachine::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (has_machine_name()) {
      if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
        machine_name_->clear();
      }
    }
    puck_id_ = 0u;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool RemovePuckFromMachine::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string machine_name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_machine_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8String(
            this->machine_name().data(), this->machine_name().length(),
            ::google::protobuf::internal::WireFormat::PARSE);
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(16)) goto parse_puck_id;
        break;
      }
      
      // required uint32 puck_id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_puck_id:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &puck_id_)));
          set_has_puck_id();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void RemovePuckFromMachine::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->machine_name(), output);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->puck_id(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* RemovePuckFromMachine::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->machine_name(), target);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->puck_id(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int RemovePuckFromMachine::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string machine_name = 1;
    if (has_machine_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->machine_name());
    }
    
    // required uint32 puck_id = 2;
    if (has_puck_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->puck_id());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void RemovePuckFromMachine::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const RemovePuckFromMachine* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const RemovePuckFromMachine*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void RemovePuckFromMachine::MergeFrom(const RemovePuckFromMachine& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_machine_name()) {
      set_machine_name(from.machine_name());
    }
    if (from.has_puck_id()) {
      set_puck_id(from.puck_id());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void RemovePuckFromMachine::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RemovePuckFromMachine::CopyFrom(const RemovePuckFromMachine& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RemovePuckFromMachine::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void RemovePuckFromMachine::Swap(RemovePuckFromMachine* other) {
  if (other != this) {
    std::swap(machine_name_, other->machine_name_);
    std::swap(puck_id_, other->puck_id_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata RemovePuckFromMachine::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = RemovePuckFromMachine_descriptor_;
  metadata.reflection = RemovePuckFromMachine_reflection_;
  return metadata;
}


// ===================================================================

const ::google::protobuf::EnumDescriptor* PlacePuckUnderMachine_CompType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PlacePuckUnderMachine_CompType_descriptor_;
}
bool PlacePuckUnderMachine_CompType_IsValid(int value) {
  switch(value) {
    case 15:
    case 2000:
      return true;
    default:
      return false;
  }
}

#ifndef _MSC_VER
const PlacePuckUnderMachine_CompType PlacePuckUnderMachine::COMP_ID;
const PlacePuckUnderMachine_CompType PlacePuckUnderMachine::MSG_TYPE;
const PlacePuckUnderMachine_CompType PlacePuckUnderMachine::CompType_MIN;
const PlacePuckUnderMachine_CompType PlacePuckUnderMachine::CompType_MAX;
const int PlacePuckUnderMachine::CompType_ARRAYSIZE;
#endif  // _MSC_VER
#ifndef _MSC_VER
const int PlacePuckUnderMachine::kMachineNameFieldNumber;
const int PlacePuckUnderMachine::kPuckIdFieldNumber;
#endif  // !_MSC_VER

PlacePuckUnderMachine::PlacePuckUnderMachine()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void PlacePuckUnderMachine::InitAsDefaultInstance() {
}

PlacePuckUnderMachine::PlacePuckUnderMachine(const PlacePuckUnderMachine& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void PlacePuckUnderMachine::SharedCtor() {
  _cached_size_ = 0;
  machine_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  puck_id_ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

PlacePuckUnderMachine::~PlacePuckUnderMachine() {
  SharedDtor();
}

void PlacePuckUnderMachine::SharedDtor() {
  if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
    delete machine_name_;
  }
  if (this != default_instance_) {
  }
}

void PlacePuckUnderMachine::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* PlacePuckUnderMachine::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PlacePuckUnderMachine_descriptor_;
}

const PlacePuckUnderMachine& PlacePuckUnderMachine::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_MachineCommands_2eproto();  return *default_instance_;
}

PlacePuckUnderMachine* PlacePuckUnderMachine::default_instance_ = NULL;

PlacePuckUnderMachine* PlacePuckUnderMachine::New() const {
  return new PlacePuckUnderMachine;
}

void PlacePuckUnderMachine::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (has_machine_name()) {
      if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
        machine_name_->clear();
      }
    }
    puck_id_ = 0u;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool PlacePuckUnderMachine::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string machine_name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_machine_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8String(
            this->machine_name().data(), this->machine_name().length(),
            ::google::protobuf::internal::WireFormat::PARSE);
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(16)) goto parse_puck_id;
        break;
      }
      
      // required uint32 puck_id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_puck_id:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &puck_id_)));
          set_has_puck_id();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void PlacePuckUnderMachine::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->machine_name(), output);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->puck_id(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* PlacePuckUnderMachine::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->machine_name(), target);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->puck_id(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int PlacePuckUnderMachine::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string machine_name = 1;
    if (has_machine_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->machine_name());
    }
    
    // required uint32 puck_id = 2;
    if (has_puck_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->puck_id());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void PlacePuckUnderMachine::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const PlacePuckUnderMachine* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const PlacePuckUnderMachine*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void PlacePuckUnderMachine::MergeFrom(const PlacePuckUnderMachine& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_machine_name()) {
      set_machine_name(from.machine_name());
    }
    if (from.has_puck_id()) {
      set_puck_id(from.puck_id());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void PlacePuckUnderMachine::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PlacePuckUnderMachine::CopyFrom(const PlacePuckUnderMachine& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PlacePuckUnderMachine::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void PlacePuckUnderMachine::Swap(PlacePuckUnderMachine* other) {
  if (other != this) {
    std::swap(machine_name_, other->machine_name_);
    std::swap(puck_id_, other->puck_id_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata PlacePuckUnderMachine::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = PlacePuckUnderMachine_descriptor_;
  metadata.reflection = PlacePuckUnderMachine_reflection_;
  return metadata;
}


// ===================================================================

const ::google::protobuf::EnumDescriptor* LoadPuckInMachine_CompType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return LoadPuckInMachine_CompType_descriptor_;
}
bool LoadPuckInMachine_CompType_IsValid(int value) {
  switch(value) {
    case 16:
    case 2000:
      return true;
    default:
      return false;
  }
}

#ifndef _MSC_VER
const LoadPuckInMachine_CompType LoadPuckInMachine::COMP_ID;
const LoadPuckInMachine_CompType LoadPuckInMachine::MSG_TYPE;
const LoadPuckInMachine_CompType LoadPuckInMachine::CompType_MIN;
const LoadPuckInMachine_CompType LoadPuckInMachine::CompType_MAX;
const int LoadPuckInMachine::CompType_ARRAYSIZE;
#endif  // _MSC_VER
#ifndef _MSC_VER
const int LoadPuckInMachine::kMachineNameFieldNumber;
const int LoadPuckInMachine::kPuckIdFieldNumber;
#endif  // !_MSC_VER

LoadPuckInMachine::LoadPuckInMachine()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void LoadPuckInMachine::InitAsDefaultInstance() {
}

LoadPuckInMachine::LoadPuckInMachine(const LoadPuckInMachine& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void LoadPuckInMachine::SharedCtor() {
  _cached_size_ = 0;
  machine_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  puck_id_ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

LoadPuckInMachine::~LoadPuckInMachine() {
  SharedDtor();
}

void LoadPuckInMachine::SharedDtor() {
  if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
    delete machine_name_;
  }
  if (this != default_instance_) {
  }
}

void LoadPuckInMachine::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LoadPuckInMachine::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return LoadPuckInMachine_descriptor_;
}

const LoadPuckInMachine& LoadPuckInMachine::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_MachineCommands_2eproto();  return *default_instance_;
}

LoadPuckInMachine* LoadPuckInMachine::default_instance_ = NULL;

LoadPuckInMachine* LoadPuckInMachine::New() const {
  return new LoadPuckInMachine;
}

void LoadPuckInMachine::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (has_machine_name()) {
      if (machine_name_ != &::google::protobuf::internal::kEmptyString) {
        machine_name_->clear();
      }
    }
    puck_id_ = 0u;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool LoadPuckInMachine::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string machine_name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_machine_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8String(
            this->machine_name().data(), this->machine_name().length(),
            ::google::protobuf::internal::WireFormat::PARSE);
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(16)) goto parse_puck_id;
        break;
      }
      
      // required uint32 puck_id = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_puck_id:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &puck_id_)));
          set_has_puck_id();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }
      
      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void LoadPuckInMachine::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->machine_name(), output);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->puck_id(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* LoadPuckInMachine::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string machine_name = 1;
  if (has_machine_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->machine_name().data(), this->machine_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->machine_name(), target);
  }
  
  // required uint32 puck_id = 2;
  if (has_puck_id()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->puck_id(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int LoadPuckInMachine::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string machine_name = 1;
    if (has_machine_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->machine_name());
    }
    
    // required uint32 puck_id = 2;
    if (has_puck_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->puck_id());
    }
    
  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LoadPuckInMachine::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const LoadPuckInMachine* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const LoadPuckInMachine*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void LoadPuckInMachine::MergeFrom(const LoadPuckInMachine& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_machine_name()) {
      set_machine_name(from.machine_name());
    }
    if (from.has_puck_id()) {
      set_puck_id(from.puck_id());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void LoadPuckInMachine::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LoadPuckInMachine::CopyFrom(const LoadPuckInMachine& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LoadPuckInMachine::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  
  return true;
}

void LoadPuckInMachine::Swap(LoadPuckInMachine* other) {
  if (other != this) {
    std::swap(machine_name_, other->machine_name_);
    std::swap(puck_id_, other->puck_id_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata LoadPuckInMachine::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = LoadPuckInMachine_descriptor_;
  metadata.reflection = LoadPuckInMachine_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace llsf_msgs

// @@protoc_insertion_point(global_scope)
