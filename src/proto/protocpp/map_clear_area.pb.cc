// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map_clear_area.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "map_clear_area.pb.h"

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
class ClearAreaDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<ClearArea>
     _instance;
} _ClearArea_default_instance_;

namespace protobuf_map_5fclear_5farea_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ClearArea, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ClearArea, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ClearArea, id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ClearArea, overlap_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ClearArea, polygon_),
  0,
  ~0u,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(ClearArea)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_ClearArea_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "map_clear_area.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  ::apollo::hdmap::protobuf_map_5fgeometry_2eproto::InitDefaults();
  _ClearArea_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_ClearArea_default_instance_);_ClearArea_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  _ClearArea_default_instance_._instance.get_mutable()->polygon_ = const_cast< ::apollo::hdmap::Polygon*>(
      ::apollo::hdmap::Polygon::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\024map_clear_area.proto\022\014apollo.hdmap\032\014ma"
      "p_id.proto\032\022map_geometry.proto\"w\n\tClearA"
      "rea\022\034\n\002id\030\001 \001(\0132\020.apollo.hdmap.Id\022$\n\nove"
      "rlap_id\030\002 \003(\0132\020.apollo.hdmap.Id\022&\n\007polyg"
      "on\030\003 \001(\0132\025.apollo.hdmap.Polygon"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 191);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "map_clear_area.proto", &protobuf_RegisterTypes);
  ::apollo::hdmap::protobuf_map_5fid_2eproto::AddDescriptors();
  ::apollo::hdmap::protobuf_map_5fgeometry_2eproto::AddDescriptors();
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

}  // namespace protobuf_map_5fclear_5farea_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ClearArea::kIdFieldNumber;
const int ClearArea::kOverlapIdFieldNumber;
const int ClearArea::kPolygonFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ClearArea::ClearArea()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_map_5fclear_5farea_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.ClearArea)
}
ClearArea::ClearArea(const ClearArea& from)
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
  if (from.has_polygon()) {
    polygon_ = new ::apollo::hdmap::Polygon(*from.polygon_);
  } else {
    polygon_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.ClearArea)
}

void ClearArea::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&polygon_) -
      reinterpret_cast<char*>(&id_)) + sizeof(polygon_));
}

ClearArea::~ClearArea() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.ClearArea)
  SharedDtor();
}

void ClearArea::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void ClearArea::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ClearArea::descriptor() {
  protobuf_map_5fclear_5farea_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_map_5fclear_5farea_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ClearArea& ClearArea::default_instance() {
  protobuf_map_5fclear_5farea_2eproto::InitDefaults();
  return *internal_default_instance();
}

ClearArea* ClearArea::New(::google::protobuf::Arena* arena) const {
  ClearArea* n = new ClearArea;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void ClearArea::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.ClearArea)
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
      GOOGLE_DCHECK(polygon_ != NULL);
      polygon_->::apollo::hdmap::Polygon::Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ClearArea::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.hdmap.ClearArea)
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

      // repeated .apollo.hdmap.Id overlap_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_overlap_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Polygon polygon = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_polygon()));
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
  // @@protoc_insertion_point(parse_success:apollo.hdmap.ClearArea)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.hdmap.ClearArea)
  return false;
#undef DO_
}

void ClearArea::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.hdmap.ClearArea)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->id_, output);
  }

  // repeated .apollo.hdmap.Id overlap_id = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->overlap_id(static_cast<int>(i)), output);
  }

  // optional .apollo.hdmap.Polygon polygon = 3;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, *this->polygon_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.hdmap.ClearArea)
}

::google::protobuf::uint8* ClearArea::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.ClearArea)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->id_, deterministic, target);
  }

  // repeated .apollo.hdmap.Id overlap_id = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, this->overlap_id(static_cast<int>(i)), deterministic, target);
  }

  // optional .apollo.hdmap.Polygon polygon = 3;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        3, *this->polygon_, deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.ClearArea)
  return target;
}

size_t ClearArea::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.ClearArea)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.hdmap.Id overlap_id = 2;
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

    // optional .apollo.hdmap.Polygon polygon = 3;
    if (has_polygon()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->polygon_);
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void ClearArea::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.ClearArea)
  GOOGLE_DCHECK_NE(&from, this);
  const ClearArea* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ClearArea>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.ClearArea)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.ClearArea)
    MergeFrom(*source);
  }
}

void ClearArea::MergeFrom(const ClearArea& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.ClearArea)
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
      mutable_polygon()->::apollo::hdmap::Polygon::MergeFrom(from.polygon());
    }
  }
}

void ClearArea::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.ClearArea)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ClearArea::CopyFrom(const ClearArea& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.ClearArea)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ClearArea::IsInitialized() const {
  return true;
}

void ClearArea::Swap(ClearArea* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ClearArea::InternalSwap(ClearArea* other) {
  using std::swap;
  overlap_id_.InternalSwap(&other->overlap_id_);
  swap(id_, other->id_);
  swap(polygon_, other->polygon_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata ClearArea::GetMetadata() const {
  protobuf_map_5fclear_5farea_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_map_5fclear_5farea_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// ClearArea

// optional .apollo.hdmap.Id id = 1;
bool ClearArea::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void ClearArea::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
void ClearArea::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
void ClearArea::clear_id() {
  if (id_ != NULL) id_->::apollo::hdmap::Id::Clear();
  clear_has_id();
}
const ::apollo::hdmap::Id& ClearArea::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
::apollo::hdmap::Id* ClearArea::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    id_ = new ::apollo::hdmap::Id;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.id)
  return id_;
}
::apollo::hdmap::Id* ClearArea::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ClearArea.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
void ClearArea::set_allocated_id(::apollo::hdmap::Id* id) {
  delete id_;
  id_ = id;
  if (id) {
    set_has_id();
  } else {
    clear_has_id();
  }
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ClearArea.id)
}

// repeated .apollo.hdmap.Id overlap_id = 2;
int ClearArea::overlap_id_size() const {
  return overlap_id_.size();
}
void ClearArea::clear_overlap_id() {
  overlap_id_.Clear();
}
const ::apollo::hdmap::Id& ClearArea::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_.Get(index);
}
::apollo::hdmap::Id* ClearArea::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_.Mutable(index);
}
::apollo::hdmap::Id* ClearArea::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_.Add();
}
::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
ClearArea::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.ClearArea.overlap_id)
  return &overlap_id_;
}
const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
ClearArea::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_;
}

// optional .apollo.hdmap.Polygon polygon = 3;
bool ClearArea::has_polygon() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void ClearArea::set_has_polygon() {
  _has_bits_[0] |= 0x00000002u;
}
void ClearArea::clear_has_polygon() {
  _has_bits_[0] &= ~0x00000002u;
}
void ClearArea::clear_polygon() {
  if (polygon_ != NULL) polygon_->::apollo::hdmap::Polygon::Clear();
  clear_has_polygon();
}
const ::apollo::hdmap::Polygon& ClearArea::polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.polygon)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Polygon*>(
      &::apollo::hdmap::_Polygon_default_instance_);
}
::apollo::hdmap::Polygon* ClearArea::mutable_polygon() {
  set_has_polygon();
  if (polygon_ == NULL) {
    polygon_ = new ::apollo::hdmap::Polygon;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.polygon)
  return polygon_;
}
::apollo::hdmap::Polygon* ClearArea::release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ClearArea.polygon)
  clear_has_polygon();
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = NULL;
  return temp;
}
void ClearArea::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
  delete polygon_;
  polygon_ = polygon;
  if (polygon) {
    set_has_polygon();
  } else {
    clear_has_polygon();
  }
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ClearArea.polygon)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)
