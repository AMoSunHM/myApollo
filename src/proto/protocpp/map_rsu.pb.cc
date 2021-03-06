// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map_rsu.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "map_rsu.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace apollo {
namespace hdmap {
class RSUDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<RSU>
     _instance;
} _RSU_default_instance_;

namespace protobuf_map_5frsu_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RSU, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RSU, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RSU, id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RSU, junction_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RSU, overlap_id_),
  0,
  1,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(RSU)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_RSU_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "map_rsu.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  ::apollo::hdmap::protobuf_map_5fid_2eproto::InitDefaults();
  _RSU_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_RSU_default_instance_);_RSU_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  _RSU_default_instance_._instance.get_mutable()->junction_id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\rmap_rsu.proto\022\014apollo.hdmap\032\014map_id.pr"
      "oto\"p\n\003RSU\022\034\n\002id\030\001 \001(\0132\020.apollo.hdmap.Id"
      "\022%\n\013junction_id\030\002 \001(\0132\020.apollo.hdmap.Id\022"
      "$\n\noverlap_id\030\003 \003(\0132\020.apollo.hdmap.Id"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 157);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "map_rsu.proto", &protobuf_RegisterTypes);
  ::apollo::hdmap::protobuf_map_5fid_2eproto::AddDescriptors();
}
} // anonymous namespace

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_map_5frsu_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RSU::kIdFieldNumber;
const int RSU::kJunctionIdFieldNumber;
const int RSU::kOverlapIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RSU::RSU()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_map_5frsu_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.RSU)
}
RSU::RSU(const RSU& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = NULL;
  }
  if (from.has_junction_id()) {
    junction_id_ = new ::apollo::hdmap::Id(*from.junction_id_);
  } else {
    junction_id_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.RSU)
}

void RSU::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&junction_id_) -
      reinterpret_cast<char*>(&id_)) + sizeof(junction_id_));
}

RSU::~RSU() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.RSU)
  SharedDtor();
}

void RSU::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete junction_id_;
}

void RSU::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* RSU::descriptor() {
  protobuf_map_5frsu_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_map_5frsu_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const RSU& RSU::default_instance() {
  protobuf_map_5frsu_2eproto::InitDefaults();
  return *internal_default_instance();
}

RSU* RSU::New(::google::protobuf::Arena* arena) const {
  RSU* n = new RSU;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void RSU::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.RSU)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != NULL);
      id_->::apollo::hdmap::Id::Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(junction_id_ != NULL);
      junction_id_->::apollo::hdmap::Id::Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool RSU::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.hdmap.RSU)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.hdmap.Id id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Id junction_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_junction_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_overlap_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.hdmap.RSU)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.hdmap.RSU)
  return false;
#undef DO_
}

void RSU::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.hdmap.RSU)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->id_, output);
  }

  // optional .apollo.hdmap.Id junction_id = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->junction_id_, output);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->overlap_id(static_cast<int>(i)), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.hdmap.RSU)
}

::google::protobuf::uint8* RSU::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.RSU)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->id_, deterministic, target);
  }

  // optional .apollo.hdmap.Id junction_id = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, *this->junction_id_, deterministic, target);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        3, this->overlap_id(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.RSU)
  return target;
}

size_t RSU::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.RSU)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.hdmap.Id overlap_id = 3;
  {
    unsigned int count = static_cast<unsigned int>(this->overlap_id_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->overlap_id(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 3u) {
    // optional .apollo.hdmap.Id id = 1;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->id_);
    }

    // optional .apollo.hdmap.Id junction_id = 2;
    if (has_junction_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->junction_id_);
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void RSU::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.RSU)
  GOOGLE_DCHECK_NE(&from, this);
  const RSU* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const RSU>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.RSU)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.RSU)
    MergeFrom(*source);
  }
}

void RSU::MergeFrom(const RSU& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.RSU)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_id()->::apollo::hdmap::Id::MergeFrom(from.id());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_junction_id()->::apollo::hdmap::Id::MergeFrom(from.junction_id());
    }
  }
}

void RSU::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.RSU)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RSU::CopyFrom(const RSU& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.RSU)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RSU::IsInitialized() const {
  return true;
}

void RSU::Swap(RSU* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RSU::InternalSwap(RSU* other) {
  using std::swap;
  overlap_id_.InternalSwap(&other->overlap_id_);
  swap(id_, other->id_);
  swap(junction_id_, other->junction_id_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata RSU::GetMetadata() const {
  protobuf_map_5frsu_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_map_5frsu_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// RSU

// optional .apollo.hdmap.Id id = 1;
bool RSU::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void RSU::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
void RSU::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
void RSU::clear_id() {
  if (id_ != NULL) id_->::apollo::hdmap::Id::Clear();
  clear_has_id();
}
const ::apollo::hdmap::Id& RSU::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
::apollo::hdmap::Id* RSU::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    id_ = new ::apollo::hdmap::Id;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.id)
  return id_;
}
::apollo::hdmap::Id* RSU::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.RSU.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
void RSU::set_allocated_id(::apollo::hdmap::Id* id) {
  delete id_;
  id_ = id;
  if (id) {
    set_has_id();
  } else {
    clear_has_id();
  }
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.RSU.id)
}

// optional .apollo.hdmap.Id junction_id = 2;
bool RSU::has_junction_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void RSU::set_has_junction_id() {
  _has_bits_[0] |= 0x00000002u;
}
void RSU::clear_has_junction_id() {
  _has_bits_[0] &= ~0x00000002u;
}
void RSU::clear_junction_id() {
  if (junction_id_ != NULL) junction_id_->::apollo::hdmap::Id::Clear();
  clear_has_junction_id();
}
const ::apollo::hdmap::Id& RSU::junction_id() const {
  const ::apollo::hdmap::Id* p = junction_id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.junction_id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
::apollo::hdmap::Id* RSU::mutable_junction_id() {
  set_has_junction_id();
  if (junction_id_ == NULL) {
    junction_id_ = new ::apollo::hdmap::Id;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.junction_id)
  return junction_id_;
}
::apollo::hdmap::Id* RSU::release_junction_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.RSU.junction_id)
  clear_has_junction_id();
  ::apollo::hdmap::Id* temp = junction_id_;
  junction_id_ = NULL;
  return temp;
}
void RSU::set_allocated_junction_id(::apollo::hdmap::Id* junction_id) {
  delete junction_id_;
  junction_id_ = junction_id;
  if (junction_id) {
    set_has_junction_id();
  } else {
    clear_has_junction_id();
  }
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.RSU.junction_id)
}

// repeated .apollo.hdmap.Id overlap_id = 3;
int RSU::overlap_id_size() const {
  return overlap_id_.size();
}
void RSU::clear_overlap_id() {
  overlap_id_.Clear();
}
const ::apollo::hdmap::Id& RSU::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Get(index);
}
::apollo::hdmap::Id* RSU::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Mutable(index);
}
::apollo::hdmap::Id* RSU::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Add();
}
::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
RSU::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.RSU.overlap_id)
  return &overlap_id_;
}
const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
RSU::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.RSU.overlap_id)
  return overlap_id_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)
