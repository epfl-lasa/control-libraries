// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: state_representation/geometry/ellipsoid.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fgeometry_2fellipsoid_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fgeometry_2fellipsoid_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3017000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3017000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "state_representation/geometry/shape.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_state_5frepresentation_2fgeometry_2fellipsoid_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_state_5frepresentation_2fgeometry_2fellipsoid_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_state_5frepresentation_2fgeometry_2fellipsoid_2eproto;
namespace state_representation {
namespace proto {
class Ellipsoid;
struct EllipsoidDefaultTypeInternal;
extern EllipsoidDefaultTypeInternal _Ellipsoid_default_instance_;
}  // namespace proto
}  // namespace state_representation
PROTOBUF_NAMESPACE_OPEN
template<> ::state_representation::proto::Ellipsoid* Arena::CreateMaybeMessage<::state_representation::proto::Ellipsoid>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace state_representation {
namespace proto {

// ===================================================================

class Ellipsoid final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:state_representation.proto.Ellipsoid) */ {
 public:
  inline Ellipsoid() : Ellipsoid(nullptr) {}
  ~Ellipsoid() override;
  explicit constexpr Ellipsoid(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Ellipsoid(const Ellipsoid& from);
  Ellipsoid(Ellipsoid&& from) noexcept
    : Ellipsoid() {
    *this = ::std::move(from);
  }

  inline Ellipsoid& operator=(const Ellipsoid& from) {
    CopyFrom(from);
    return *this;
  }
  inline Ellipsoid& operator=(Ellipsoid&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Ellipsoid& default_instance() {
    return *internal_default_instance();
  }
  static inline const Ellipsoid* internal_default_instance() {
    return reinterpret_cast<const Ellipsoid*>(
               &_Ellipsoid_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Ellipsoid& a, Ellipsoid& b) {
    a.Swap(&b);
  }
  inline void Swap(Ellipsoid* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Ellipsoid* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Ellipsoid* New() const final {
    return new Ellipsoid();
  }

  Ellipsoid* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Ellipsoid>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Ellipsoid& from);
  void MergeFrom(const Ellipsoid& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Ellipsoid* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "state_representation.proto.Ellipsoid";
  }
  protected:
  explicit Ellipsoid(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAxisLengthsFieldNumber = 2,
    kShapeFieldNumber = 1,
    kRotationAngleFieldNumber = 3,
  };
  // repeated double axis_lengths = 2;
  int axis_lengths_size() const;
  private:
  int _internal_axis_lengths_size() const;
  public:
  void clear_axis_lengths();
  private:
  double _internal_axis_lengths(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_axis_lengths() const;
  void _internal_add_axis_lengths(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_axis_lengths();
  public:
  double axis_lengths(int index) const;
  void set_axis_lengths(int index, double value);
  void add_axis_lengths(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      axis_lengths() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_axis_lengths();

  // .state_representation.proto.Shape shape = 1;
  bool has_shape() const;
  private:
  bool _internal_has_shape() const;
  public:
  void clear_shape();
  const ::state_representation::proto::Shape& shape() const;
  PROTOBUF_FUTURE_MUST_USE_RESULT ::state_representation::proto::Shape* release_shape();
  ::state_representation::proto::Shape* mutable_shape();
  void set_allocated_shape(::state_representation::proto::Shape* shape);
  private:
  const ::state_representation::proto::Shape& _internal_shape() const;
  ::state_representation::proto::Shape* _internal_mutable_shape();
  public:
  void unsafe_arena_set_allocated_shape(
      ::state_representation::proto::Shape* shape);
  ::state_representation::proto::Shape* unsafe_arena_release_shape();

  // double rotation_angle = 3;
  void clear_rotation_angle();
  double rotation_angle() const;
  void set_rotation_angle(double value);
  private:
  double _internal_rotation_angle() const;
  void _internal_set_rotation_angle(double value);
  public:

  // @@protoc_insertion_point(class_scope:state_representation.proto.Ellipsoid)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > axis_lengths_;
  ::state_representation::proto::Shape* shape_;
  double rotation_angle_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_state_5frepresentation_2fgeometry_2fellipsoid_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Ellipsoid

// .state_representation.proto.Shape shape = 1;
inline bool Ellipsoid::_internal_has_shape() const {
  return this != internal_default_instance() && shape_ != nullptr;
}
inline bool Ellipsoid::has_shape() const {
  return _internal_has_shape();
}
inline const ::state_representation::proto::Shape& Ellipsoid::_internal_shape() const {
  const ::state_representation::proto::Shape* p = shape_;
  return p != nullptr ? *p : reinterpret_cast<const ::state_representation::proto::Shape&>(
      ::state_representation::proto::_Shape_default_instance_);
}
inline const ::state_representation::proto::Shape& Ellipsoid::shape() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Ellipsoid.shape)
  return _internal_shape();
}
inline void Ellipsoid::unsafe_arena_set_allocated_shape(
    ::state_representation::proto::Shape* shape) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(shape_);
  }
  shape_ = shape;
  if (shape) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:state_representation.proto.Ellipsoid.shape)
}
inline ::state_representation::proto::Shape* Ellipsoid::release_shape() {
  
  ::state_representation::proto::Shape* temp = shape_;
  shape_ = nullptr;
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::state_representation::proto::Shape* Ellipsoid::unsafe_arena_release_shape() {
  // @@protoc_insertion_point(field_release:state_representation.proto.Ellipsoid.shape)
  
  ::state_representation::proto::Shape* temp = shape_;
  shape_ = nullptr;
  return temp;
}
inline ::state_representation::proto::Shape* Ellipsoid::_internal_mutable_shape() {
  
  if (shape_ == nullptr) {
    auto* p = CreateMaybeMessage<::state_representation::proto::Shape>(GetArenaForAllocation());
    shape_ = p;
  }
  return shape_;
}
inline ::state_representation::proto::Shape* Ellipsoid::mutable_shape() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.Ellipsoid.shape)
  return _internal_mutable_shape();
}
inline void Ellipsoid::set_allocated_shape(::state_representation::proto::Shape* shape) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(shape_);
  }
  if (shape) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(shape));
    if (message_arena != submessage_arena) {
      shape = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, shape, submessage_arena);
    }
    
  } else {
    
  }
  shape_ = shape;
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.Ellipsoid.shape)
}

// repeated double axis_lengths = 2;
inline int Ellipsoid::_internal_axis_lengths_size() const {
  return axis_lengths_.size();
}
inline int Ellipsoid::axis_lengths_size() const {
  return _internal_axis_lengths_size();
}
inline void Ellipsoid::clear_axis_lengths() {
  axis_lengths_.Clear();
}
inline double Ellipsoid::_internal_axis_lengths(int index) const {
  return axis_lengths_.Get(index);
}
inline double Ellipsoid::axis_lengths(int index) const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Ellipsoid.axis_lengths)
  return _internal_axis_lengths(index);
}
inline void Ellipsoid::set_axis_lengths(int index, double value) {
  axis_lengths_.Set(index, value);
  // @@protoc_insertion_point(field_set:state_representation.proto.Ellipsoid.axis_lengths)
}
inline void Ellipsoid::_internal_add_axis_lengths(double value) {
  axis_lengths_.Add(value);
}
inline void Ellipsoid::add_axis_lengths(double value) {
  _internal_add_axis_lengths(value);
  // @@protoc_insertion_point(field_add:state_representation.proto.Ellipsoid.axis_lengths)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Ellipsoid::_internal_axis_lengths() const {
  return axis_lengths_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
Ellipsoid::axis_lengths() const {
  // @@protoc_insertion_point(field_list:state_representation.proto.Ellipsoid.axis_lengths)
  return _internal_axis_lengths();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Ellipsoid::_internal_mutable_axis_lengths() {
  return &axis_lengths_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
Ellipsoid::mutable_axis_lengths() {
  // @@protoc_insertion_point(field_mutable_list:state_representation.proto.Ellipsoid.axis_lengths)
  return _internal_mutable_axis_lengths();
}

// double rotation_angle = 3;
inline void Ellipsoid::clear_rotation_angle() {
  rotation_angle_ = 0;
}
inline double Ellipsoid::_internal_rotation_angle() const {
  return rotation_angle_;
}
inline double Ellipsoid::rotation_angle() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.Ellipsoid.rotation_angle)
  return _internal_rotation_angle();
}
inline void Ellipsoid::_internal_set_rotation_angle(double value) {
  
  rotation_angle_ = value;
}
inline void Ellipsoid::set_rotation_angle(double value) {
  _internal_set_rotation_angle(value);
  // @@protoc_insertion_point(field_set:state_representation.proto.Ellipsoid.rotation_angle)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace state_representation

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fgeometry_2fellipsoid_2eproto
