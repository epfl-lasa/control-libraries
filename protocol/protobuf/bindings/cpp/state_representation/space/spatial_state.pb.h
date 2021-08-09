// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: state_representation/space/spatial_state.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto

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
#include "state_representation/state.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto;
namespace state_representation {
namespace proto {
class SpatialState;
struct SpatialStateDefaultTypeInternal;
extern SpatialStateDefaultTypeInternal _SpatialState_default_instance_;
}  // namespace proto
}  // namespace state_representation
PROTOBUF_NAMESPACE_OPEN
template<> ::state_representation::proto::SpatialState* Arena::CreateMaybeMessage<::state_representation::proto::SpatialState>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace state_representation {
namespace proto {

// ===================================================================

class SpatialState final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:state_representation.proto.SpatialState) */ {
 public:
  inline SpatialState() : SpatialState(nullptr) {}
  ~SpatialState() override;
  explicit constexpr SpatialState(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SpatialState(const SpatialState& from);
  SpatialState(SpatialState&& from) noexcept
    : SpatialState() {
    *this = ::std::move(from);
  }

  inline SpatialState& operator=(const SpatialState& from) {
    CopyFrom(from);
    return *this;
  }
  inline SpatialState& operator=(SpatialState&& from) noexcept {
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
  static const SpatialState& default_instance() {
    return *internal_default_instance();
  }
  static inline const SpatialState* internal_default_instance() {
    return reinterpret_cast<const SpatialState*>(
               &_SpatialState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SpatialState& a, SpatialState& b) {
    a.Swap(&b);
  }
  inline void Swap(SpatialState* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(SpatialState* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SpatialState* New() const final {
    return new SpatialState();
  }

  SpatialState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SpatialState>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SpatialState& from);
  void MergeFrom(const SpatialState& from);
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
  void InternalSwap(SpatialState* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "state_representation.proto.SpatialState";
  }
  protected:
  explicit SpatialState(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kReferenceFrameFieldNumber = 2,
    kStateFieldNumber = 1,
  };
  // string reference_frame = 2;
  void clear_reference_frame();
  const std::string& reference_frame() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_reference_frame(ArgT0&& arg0, ArgT... args);
  std::string* mutable_reference_frame();
  PROTOBUF_FUTURE_MUST_USE_RESULT std::string* release_reference_frame();
  void set_allocated_reference_frame(std::string* reference_frame);
  private:
  const std::string& _internal_reference_frame() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_reference_frame(const std::string& value);
  std::string* _internal_mutable_reference_frame();
  public:

  // .state_representation.proto.State state = 1;
  bool has_state() const;
  private:
  bool _internal_has_state() const;
  public:
  void clear_state();
  const ::state_representation::proto::State& state() const;
  PROTOBUF_FUTURE_MUST_USE_RESULT ::state_representation::proto::State* release_state();
  ::state_representation::proto::State* mutable_state();
  void set_allocated_state(::state_representation::proto::State* state);
  private:
  const ::state_representation::proto::State& _internal_state() const;
  ::state_representation::proto::State* _internal_mutable_state();
  public:
  void unsafe_arena_set_allocated_state(
      ::state_representation::proto::State* state);
  ::state_representation::proto::State* unsafe_arena_release_state();

  // @@protoc_insertion_point(class_scope:state_representation.proto.SpatialState)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr reference_frame_;
  ::state_representation::proto::State* state_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SpatialState

// .state_representation.proto.State state = 1;
inline bool SpatialState::_internal_has_state() const {
  return this != internal_default_instance() && state_ != nullptr;
}
inline bool SpatialState::has_state() const {
  return _internal_has_state();
}
inline const ::state_representation::proto::State& SpatialState::_internal_state() const {
  const ::state_representation::proto::State* p = state_;
  return p != nullptr ? *p : reinterpret_cast<const ::state_representation::proto::State&>(
      ::state_representation::proto::_State_default_instance_);
}
inline const ::state_representation::proto::State& SpatialState::state() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.SpatialState.state)
  return _internal_state();
}
inline void SpatialState::unsafe_arena_set_allocated_state(
    ::state_representation::proto::State* state) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(state_);
  }
  state_ = state;
  if (state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:state_representation.proto.SpatialState.state)
}
inline ::state_representation::proto::State* SpatialState::release_state() {
  
  ::state_representation::proto::State* temp = state_;
  state_ = nullptr;
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::state_representation::proto::State* SpatialState::unsafe_arena_release_state() {
  // @@protoc_insertion_point(field_release:state_representation.proto.SpatialState.state)
  
  ::state_representation::proto::State* temp = state_;
  state_ = nullptr;
  return temp;
}
inline ::state_representation::proto::State* SpatialState::_internal_mutable_state() {
  
  if (state_ == nullptr) {
    auto* p = CreateMaybeMessage<::state_representation::proto::State>(GetArenaForAllocation());
    state_ = p;
  }
  return state_;
}
inline ::state_representation::proto::State* SpatialState::mutable_state() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.SpatialState.state)
  return _internal_mutable_state();
}
inline void SpatialState::set_allocated_state(::state_representation::proto::State* state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(state_);
  }
  if (state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(state));
    if (message_arena != submessage_arena) {
      state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, state, submessage_arena);
    }
    
  } else {
    
  }
  state_ = state;
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.SpatialState.state)
}

// string reference_frame = 2;
inline void SpatialState::clear_reference_frame() {
  reference_frame_.ClearToEmpty();
}
inline const std::string& SpatialState::reference_frame() const {
  // @@protoc_insertion_point(field_get:state_representation.proto.SpatialState.reference_frame)
  return _internal_reference_frame();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void SpatialState::set_reference_frame(ArgT0&& arg0, ArgT... args) {
 
 reference_frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:state_representation.proto.SpatialState.reference_frame)
}
inline std::string* SpatialState::mutable_reference_frame() {
  // @@protoc_insertion_point(field_mutable:state_representation.proto.SpatialState.reference_frame)
  return _internal_mutable_reference_frame();
}
inline const std::string& SpatialState::_internal_reference_frame() const {
  return reference_frame_.Get();
}
inline void SpatialState::_internal_set_reference_frame(const std::string& value) {
  
  reference_frame_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* SpatialState::_internal_mutable_reference_frame() {
  
  return reference_frame_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* SpatialState::release_reference_frame() {
  // @@protoc_insertion_point(field_release:state_representation.proto.SpatialState.reference_frame)
  return reference_frame_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void SpatialState::set_allocated_reference_frame(std::string* reference_frame) {
  if (reference_frame != nullptr) {
    
  } else {
    
  }
  reference_frame_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), reference_frame,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:state_representation.proto.SpatialState.reference_frame)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace state_representation

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_state_5frepresentation_2fspace_2fspatial_5fstate_2eproto
