// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: SimTimeSync.proto

#ifndef PROTOBUF_SimTimeSync_2eproto__INCLUDED
#define PROTOBUF_SimTimeSync_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2004001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>
#include "Time.pb.h"
// @@protoc_insertion_point(includes)

namespace llsf_msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_SimTimeSync_2eproto();
void protobuf_AssignDesc_SimTimeSync_2eproto();
void protobuf_ShutdownFile_SimTimeSync_2eproto();

class SimTimeSync;

enum SimTimeSync_CompType {
  SimTimeSync_CompType_COMP_ID = 2000,
  SimTimeSync_CompType_MSG_TYPE = 327
};
bool SimTimeSync_CompType_IsValid(int value);
const SimTimeSync_CompType SimTimeSync_CompType_CompType_MIN = SimTimeSync_CompType_MSG_TYPE;
const SimTimeSync_CompType SimTimeSync_CompType_CompType_MAX = SimTimeSync_CompType_COMP_ID;
const int SimTimeSync_CompType_CompType_ARRAYSIZE = SimTimeSync_CompType_CompType_MAX + 1;

const ::google::protobuf::EnumDescriptor* SimTimeSync_CompType_descriptor();
inline const ::std::string& SimTimeSync_CompType_Name(SimTimeSync_CompType value) {
  return ::google::protobuf::internal::NameOfEnum(
    SimTimeSync_CompType_descriptor(), value);
}
inline bool SimTimeSync_CompType_Parse(
    const ::std::string& name, SimTimeSync_CompType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SimTimeSync_CompType>(
    SimTimeSync_CompType_descriptor(), name, value);
}
// ===================================================================

class SimTimeSync : public ::google::protobuf::Message {
 public:
  SimTimeSync();
  virtual ~SimTimeSync();
  
  SimTimeSync(const SimTimeSync& from);
  
  inline SimTimeSync& operator=(const SimTimeSync& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const SimTimeSync& default_instance();
  
  void Swap(SimTimeSync* other);
  
  // implements Message ----------------------------------------------
  
  SimTimeSync* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SimTimeSync& from);
  void MergeFrom(const SimTimeSync& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  typedef SimTimeSync_CompType CompType;
  static const CompType COMP_ID = SimTimeSync_CompType_COMP_ID;
  static const CompType MSG_TYPE = SimTimeSync_CompType_MSG_TYPE;
  static inline bool CompType_IsValid(int value) {
    return SimTimeSync_CompType_IsValid(value);
  }
  static const CompType CompType_MIN =
    SimTimeSync_CompType_CompType_MIN;
  static const CompType CompType_MAX =
    SimTimeSync_CompType_CompType_MAX;
  static const int CompType_ARRAYSIZE =
    SimTimeSync_CompType_CompType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CompType_descriptor() {
    return SimTimeSync_CompType_descriptor();
  }
  static inline const ::std::string& CompType_Name(CompType value) {
    return SimTimeSync_CompType_Name(value);
  }
  static inline bool CompType_Parse(const ::std::string& name,
      CompType* value) {
    return SimTimeSync_CompType_Parse(name, value);
  }
  
  // accessors -------------------------------------------------------
  
  // required .llsf_msgs.Time sim_time = 1;
  inline bool has_sim_time() const;
  inline void clear_sim_time();
  static const int kSimTimeFieldNumber = 1;
  inline const ::llsf_msgs::Time& sim_time() const;
  inline ::llsf_msgs::Time* mutable_sim_time();
  inline ::llsf_msgs::Time* release_sim_time();
  
  // required float real_time_factor = 2;
  inline bool has_real_time_factor() const;
  inline void clear_real_time_factor();
  static const int kRealTimeFactorFieldNumber = 2;
  inline float real_time_factor() const;
  inline void set_real_time_factor(float value);
  
  // required bool paused = 3;
  inline bool has_paused() const;
  inline void clear_paused();
  static const int kPausedFieldNumber = 3;
  inline bool paused() const;
  inline void set_paused(bool value);
  
  // @@protoc_insertion_point(class_scope:llsf_msgs.SimTimeSync)
 private:
  inline void set_has_sim_time();
  inline void clear_has_sim_time();
  inline void set_has_real_time_factor();
  inline void clear_has_real_time_factor();
  inline void set_has_paused();
  inline void clear_has_paused();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::llsf_msgs::Time* sim_time_;
  float real_time_factor_;
  bool paused_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  friend void  protobuf_AddDesc_SimTimeSync_2eproto();
  friend void protobuf_AssignDesc_SimTimeSync_2eproto();
  friend void protobuf_ShutdownFile_SimTimeSync_2eproto();
  
  void InitAsDefaultInstance();
  static SimTimeSync* default_instance_;
};
// ===================================================================


// ===================================================================

// SimTimeSync

// required .llsf_msgs.Time sim_time = 1;
inline bool SimTimeSync::has_sim_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SimTimeSync::set_has_sim_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SimTimeSync::clear_has_sim_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SimTimeSync::clear_sim_time() {
  if (sim_time_ != NULL) sim_time_->::llsf_msgs::Time::Clear();
  clear_has_sim_time();
}
inline const ::llsf_msgs::Time& SimTimeSync::sim_time() const {
  return sim_time_ != NULL ? *sim_time_ : *default_instance_->sim_time_;
}
inline ::llsf_msgs::Time* SimTimeSync::mutable_sim_time() {
  set_has_sim_time();
  if (sim_time_ == NULL) sim_time_ = new ::llsf_msgs::Time;
  return sim_time_;
}
inline ::llsf_msgs::Time* SimTimeSync::release_sim_time() {
  clear_has_sim_time();
  ::llsf_msgs::Time* temp = sim_time_;
  sim_time_ = NULL;
  return temp;
}

// required float real_time_factor = 2;
inline bool SimTimeSync::has_real_time_factor() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SimTimeSync::set_has_real_time_factor() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SimTimeSync::clear_has_real_time_factor() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SimTimeSync::clear_real_time_factor() {
  real_time_factor_ = 0;
  clear_has_real_time_factor();
}
inline float SimTimeSync::real_time_factor() const {
  return real_time_factor_;
}
inline void SimTimeSync::set_real_time_factor(float value) {
  set_has_real_time_factor();
  real_time_factor_ = value;
}

// required bool paused = 3;
inline bool SimTimeSync::has_paused() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SimTimeSync::set_has_paused() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SimTimeSync::clear_has_paused() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SimTimeSync::clear_paused() {
  paused_ = false;
  clear_has_paused();
}
inline bool SimTimeSync::paused() const {
  return paused_;
}
inline void SimTimeSync::set_paused(bool value) {
  set_has_paused();
  paused_ = value;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace llsf_msgs

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::llsf_msgs::SimTimeSync_CompType>() {
  return ::llsf_msgs::SimTimeSync_CompType_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_SimTimeSync_2eproto__INCLUDED