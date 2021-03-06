// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map_yield_sign.proto

#ifndef PROTOBUF_map_5fyield_5fsign_2eproto__INCLUDED
#define PROTOBUF_map_5fyield_5fsign_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "map_id.pb.h"
#include "map_geometry.pb.h"
// @@protoc_insertion_point(includes)
namespace apollo {
namespace hdmap {
class YieldSign;
class YieldSignDefaultTypeInternal;
extern YieldSignDefaultTypeInternal _YieldSign_default_instance_;
}  // namespace hdmap
}  // namespace apollo

namespace apollo {
namespace hdmap {

namespace protobuf_map_5fyield_5fsign_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_map_5fyield_5fsign_2eproto

// ===================================================================

class YieldSign : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.YieldSign) */ {
 public:
  YieldSign();
  virtual ~YieldSign();

  YieldSign(const YieldSign& from);

  inline YieldSign& operator=(const YieldSign& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  YieldSign(YieldSign&& from) noexcept
    : YieldSign() {
    *this = ::std::move(from);
  }

  inline YieldSign& operator=(YieldSign&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const YieldSign& default_instance();

  static inline const YieldSign* internal_default_instance() {
    return reinterpret_cast<const YieldSign*>(
               &_YieldSign_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(YieldSign* other);
  friend void swap(YieldSign& a, YieldSign& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline YieldSign* New() const PROTOBUF_FINAL { return New(NULL); }

  YieldSign* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const YieldSign& from);
  void MergeFrom(const YieldSign& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(YieldSign* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .apollo.hdmap.Curve stop_line = 2;
  int stop_line_size() const;
  void clear_stop_line();
  static const int kStopLineFieldNumber = 2;
  const ::apollo::hdmap::Curve& stop_line(int index) const;
  ::apollo::hdmap::Curve* mutable_stop_line(int index);
  ::apollo::hdmap::Curve* add_stop_line();
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >*
      mutable_stop_line();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >&
      stop_line() const;

  // repeated .apollo.hdmap.Id overlap_id = 3;
  int overlap_id_size() const;
  void clear_overlap_id();
  static const int kOverlapIdFieldNumber = 3;
  const ::apollo::hdmap::Id& overlap_id(int index) const;
  ::apollo::hdmap::Id* mutable_overlap_id(int index);
  ::apollo::hdmap::Id* add_overlap_id();
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_overlap_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      overlap_id() const;

  // optional .apollo.hdmap.Id id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  const ::apollo::hdmap::Id& id() const;
  ::apollo::hdmap::Id* mutable_id();
  ::apollo::hdmap::Id* release_id();
  void set_allocated_id(::apollo::hdmap::Id* id);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.YieldSign)
 private:
  void set_has_id();
  void clear_has_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve > stop_line_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  friend struct protobuf_map_5fyield_5fsign_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// YieldSign

// optional .apollo.hdmap.Id id = 1;
inline bool YieldSign::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void YieldSign::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void YieldSign::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void YieldSign::clear_id() {
  if (id_ != NULL) id_->::apollo::hdmap::Id::Clear();
  clear_has_id();
}
inline const ::apollo::hdmap::Id& YieldSign::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* YieldSign::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    id_ = new ::apollo::hdmap::Id;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.id)
  return id_;
}
inline ::apollo::hdmap::Id* YieldSign::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.YieldSign.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline void YieldSign::set_allocated_id(::apollo::hdmap::Id* id) {
  delete id_;
  id_ = id;
  if (id) {
    set_has_id();
  } else {
    clear_has_id();
  }
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.YieldSign.id)
}

// repeated .apollo.hdmap.Curve stop_line = 2;
inline int YieldSign::stop_line_size() const {
  return stop_line_.size();
}
inline void YieldSign::clear_stop_line() {
  stop_line_.Clear();
}
inline const ::apollo::hdmap::Curve& YieldSign::stop_line(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Get(index);
}
inline ::apollo::hdmap::Curve* YieldSign::mutable_stop_line(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Mutable(index);
}
inline ::apollo::hdmap::Curve* YieldSign::add_stop_line() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >*
YieldSign::mutable_stop_line() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.stop_line)
  return &stop_line_;
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >&
YieldSign::stop_line() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.stop_line)
  return stop_line_;
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int YieldSign::overlap_id_size() const {
  return overlap_id_.size();
}
inline void YieldSign::clear_overlap_id() {
  overlap_id_.Clear();
}
inline const ::apollo::hdmap::Id& YieldSign::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* YieldSign::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::apollo::hdmap::Id* YieldSign::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
YieldSign::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.overlap_id)
  return &overlap_id_;
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
YieldSign::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_map_5fyield_5fsign_2eproto__INCLUDED
