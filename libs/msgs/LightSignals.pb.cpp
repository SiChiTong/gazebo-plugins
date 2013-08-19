// Generated by the protocol buffer compiler.  DO NOT EDIT!

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "LightSignals.pb.h"

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

const ::google::protobuf::Descriptor* MachineSignal_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  MachineSignal_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* MachineSignal_CompType_descriptor_ = NULL;
const ::google::protobuf::Descriptor* AllMachineSignals_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  AllMachineSignals_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* AllMachineSignals_CompType_descriptor_ = NULL;

}  // namespace


void protobuf_AssignDesc_LightSignals_2eproto() {
  protobuf_AddDesc_LightSignals_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "LightSignals.proto");
  GOOGLE_CHECK(file != NULL);
  MachineSignal_descriptor_ = file->message_type(0);
  static const int MachineSignal_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MachineSignal, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MachineSignal, lights_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MachineSignal, pose_),
  };
  MachineSignal_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      MachineSignal_descriptor_,
      MachineSignal::default_instance_,
      MachineSignal_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MachineSignal, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MachineSignal, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(MachineSignal));
  MachineSignal_CompType_descriptor_ = MachineSignal_descriptor_->enum_type(0);
  AllMachineSignals_descriptor_ = file->message_type(1);
  static const int AllMachineSignals_offsets_[1] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AllMachineSignals, machines_),
  };
  AllMachineSignals_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      AllMachineSignals_descriptor_,
      AllMachineSignals::default_instance_,
      AllMachineSignals_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AllMachineSignals, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AllMachineSignals, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(AllMachineSignals));
  AllMachineSignals_CompType_descriptor_ = AllMachineSignals_descriptor_->enum_type(0);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_LightSignals_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    MachineSignal_descriptor_, &MachineSignal::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    AllMachineSignals_descriptor_, &AllMachineSignals::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_LightSignals_2eproto() {
  delete MachineSignal::default_instance_;
  delete MachineSignal_reflection_;
  delete AllMachineSignals::default_instance_;
  delete AllMachineSignals_reflection_;
}

void protobuf_AddDesc_LightSignals_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::llsf_msgs::protobuf_AddDesc_Pose2D_2eproto();
  ::llsf_msgs::protobuf_AddDesc_MachineInfo_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\022LightSignals.proto\022\tllsf_msgs\032\014Pose2D."
    "proto\032\021MachineInfo.proto\"\215\001\n\rMachineSign"
    "al\022\014\n\004name\030\001 \002(\t\022$\n\006lights\030\002 \003(\0132\024.llsf_"
    "msgs.LightSpec\022\037\n\004pose\030\003 \002(\0132\021.llsf_msgs"
    ".Pose2D\"\'\n\010CompType\022\014\n\007COMP_ID\020\320\017\022\r\n\010MSG"
    "_TYPE\020\362\001\"h\n\021AllMachineSignals\022*\n\010machine"
    "s\030\001 \003(\0132\030.llsf_msgs.MachineSignal\"\'\n\010Com"
    "pType\022\014\n\007COMP_ID\020\320\017\022\r\n\010MSG_TYPE\020\304\002", 314);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "LightSignals.proto", &protobuf_RegisterTypes);
  MachineSignal::default_instance_ = new MachineSignal();
  AllMachineSignals::default_instance_ = new AllMachineSignals();
  MachineSignal::default_instance_->InitAsDefaultInstance();
  AllMachineSignals::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_LightSignals_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_LightSignals_2eproto {
  StaticDescriptorInitializer_LightSignals_2eproto() {
    protobuf_AddDesc_LightSignals_2eproto();
  }
} static_descriptor_initializer_LightSignals_2eproto_;


// ===================================================================

const ::google::protobuf::EnumDescriptor* MachineSignal_CompType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return MachineSignal_CompType_descriptor_;
}
bool MachineSignal_CompType_IsValid(int value) {
  switch(value) {
    case 242:
    case 2000:
      return true;
    default:
      return false;
  }
}

#ifndef _MSC_VER
const MachineSignal_CompType MachineSignal::COMP_ID;
const MachineSignal_CompType MachineSignal::MSG_TYPE;
const MachineSignal_CompType MachineSignal::CompType_MIN;
const MachineSignal_CompType MachineSignal::CompType_MAX;
const int MachineSignal::CompType_ARRAYSIZE;
#endif  // _MSC_VER
#ifndef _MSC_VER
const int MachineSignal::kNameFieldNumber;
const int MachineSignal::kLightsFieldNumber;
const int MachineSignal::kPoseFieldNumber;
#endif  // !_MSC_VER

MachineSignal::MachineSignal()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void MachineSignal::InitAsDefaultInstance() {
  pose_ = const_cast< ::llsf_msgs::Pose2D*>(&::llsf_msgs::Pose2D::default_instance());
}

MachineSignal::MachineSignal(const MachineSignal& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void MachineSignal::SharedCtor() {
  _cached_size_ = 0;
  name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  pose_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

MachineSignal::~MachineSignal() {
  SharedDtor();
}

void MachineSignal::SharedDtor() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    delete name_;
  }
  if (this != default_instance_) {
    delete pose_;
  }
}

void MachineSignal::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* MachineSignal::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return MachineSignal_descriptor_;
}

const MachineSignal& MachineSignal::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_LightSignals_2eproto();  return *default_instance_;
}

MachineSignal* MachineSignal::default_instance_ = NULL;

MachineSignal* MachineSignal::New() const {
  return new MachineSignal;
}

void MachineSignal::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (has_name()) {
      if (name_ != &::google::protobuf::internal::kEmptyString) {
        name_->clear();
      }
    }
    if (has_pose()) {
      if (pose_ != NULL) pose_->::llsf_msgs::Pose2D::Clear();
    }
  }
  lights_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool MachineSignal::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string name = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8String(
            this->name().data(), this->name().length(),
            ::google::protobuf::internal::WireFormat::PARSE);
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(18)) goto parse_lights;
        break;
      }
      
      // repeated .llsf_msgs.LightSpec lights = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_lights:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_lights()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(18)) goto parse_lights;
        if (input->ExpectTag(26)) goto parse_pose;
        break;
      }
      
      // required .llsf_msgs.Pose2D pose = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_pose:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_pose()));
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

void MachineSignal::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // required string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->name(), output);
  }
  
  // repeated .llsf_msgs.LightSpec lights = 2;
  for (int i = 0; i < this->lights_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->lights(i), output);
  }
  
  // required .llsf_msgs.Pose2D pose = 3;
  if (has_pose()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->pose(), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* MachineSignal::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // required string name = 1;
  if (has_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8String(
      this->name().data(), this->name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE);
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }
  
  // repeated .llsf_msgs.LightSpec lights = 2;
  for (int i = 0; i < this->lights_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        2, this->lights(i), target);
  }
  
  // required .llsf_msgs.Pose2D pose = 3;
  if (has_pose()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->pose(), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int MachineSignal::ByteSize() const {
  int total_size = 0;
  
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required string name = 1;
    if (has_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->name());
    }
    
    // required .llsf_msgs.Pose2D pose = 3;
    if (has_pose()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->pose());
    }
    
  }
  // repeated .llsf_msgs.LightSpec lights = 2;
  total_size += 1 * this->lights_size();
  for (int i = 0; i < this->lights_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->lights(i));
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

void MachineSignal::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const MachineSignal* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const MachineSignal*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void MachineSignal::MergeFrom(const MachineSignal& from) {
  GOOGLE_CHECK_NE(&from, this);
  lights_.MergeFrom(from.lights_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_name()) {
      set_name(from.name());
    }
    if (from.has_pose()) {
      mutable_pose()->::llsf_msgs::Pose2D::MergeFrom(from.pose());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void MachineSignal::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MachineSignal::CopyFrom(const MachineSignal& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MachineSignal::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000005) != 0x00000005) return false;
  
  for (int i = 0; i < lights_size(); i++) {
    if (!this->lights(i).IsInitialized()) return false;
  }
  if (has_pose()) {
    if (!this->pose().IsInitialized()) return false;
  }
  return true;
}

void MachineSignal::Swap(MachineSignal* other) {
  if (other != this) {
    std::swap(name_, other->name_);
    lights_.Swap(&other->lights_);
    std::swap(pose_, other->pose_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata MachineSignal::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = MachineSignal_descriptor_;
  metadata.reflection = MachineSignal_reflection_;
  return metadata;
}


// ===================================================================

const ::google::protobuf::EnumDescriptor* AllMachineSignals_CompType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return AllMachineSignals_CompType_descriptor_;
}
bool AllMachineSignals_CompType_IsValid(int value) {
  switch(value) {
    case 324:
    case 2000:
      return true;
    default:
      return false;
  }
}

#ifndef _MSC_VER
const AllMachineSignals_CompType AllMachineSignals::COMP_ID;
const AllMachineSignals_CompType AllMachineSignals::MSG_TYPE;
const AllMachineSignals_CompType AllMachineSignals::CompType_MIN;
const AllMachineSignals_CompType AllMachineSignals::CompType_MAX;
const int AllMachineSignals::CompType_ARRAYSIZE;
#endif  // _MSC_VER
#ifndef _MSC_VER
const int AllMachineSignals::kMachinesFieldNumber;
#endif  // !_MSC_VER

AllMachineSignals::AllMachineSignals()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void AllMachineSignals::InitAsDefaultInstance() {
}

AllMachineSignals::AllMachineSignals(const AllMachineSignals& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void AllMachineSignals::SharedCtor() {
  _cached_size_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

AllMachineSignals::~AllMachineSignals() {
  SharedDtor();
}

void AllMachineSignals::SharedDtor() {
  if (this != default_instance_) {
  }
}

void AllMachineSignals::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* AllMachineSignals::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return AllMachineSignals_descriptor_;
}

const AllMachineSignals& AllMachineSignals::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_LightSignals_2eproto();  return *default_instance_;
}

AllMachineSignals* AllMachineSignals::default_instance_ = NULL;

AllMachineSignals* AllMachineSignals::New() const {
  return new AllMachineSignals;
}

void AllMachineSignals::Clear() {
  machines_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool AllMachineSignals::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .llsf_msgs.MachineSignal machines = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_machines:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_machines()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(10)) goto parse_machines;
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

void AllMachineSignals::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // repeated .llsf_msgs.MachineSignal machines = 1;
  for (int i = 0; i < this->machines_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->machines(i), output);
  }
  
  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* AllMachineSignals::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // repeated .llsf_msgs.MachineSignal machines = 1;
  for (int i = 0; i < this->machines_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->machines(i), target);
  }
  
  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int AllMachineSignals::ByteSize() const {
  int total_size = 0;
  
  // repeated .llsf_msgs.MachineSignal machines = 1;
  total_size += 1 * this->machines_size();
  for (int i = 0; i < this->machines_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->machines(i));
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

void AllMachineSignals::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const AllMachineSignals* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const AllMachineSignals*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void AllMachineSignals::MergeFrom(const AllMachineSignals& from) {
  GOOGLE_CHECK_NE(&from, this);
  machines_.MergeFrom(from.machines_);
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void AllMachineSignals::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AllMachineSignals::CopyFrom(const AllMachineSignals& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AllMachineSignals::IsInitialized() const {
  
  for (int i = 0; i < machines_size(); i++) {
    if (!this->machines(i).IsInitialized()) return false;
  }
  return true;
}

void AllMachineSignals::Swap(AllMachineSignals* other) {
  if (other != this) {
    machines_.Swap(&other->machines_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata AllMachineSignals::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = AllMachineSignals_descriptor_;
  metadata.reflection = AllMachineSignals_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace llsf_msgs

// @@protoc_insertion_point(global_scope)
