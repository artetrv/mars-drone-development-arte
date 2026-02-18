// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#include "ardupilot_msgs/msg/detail/rc__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
const rosidl_type_hash_t *
ardupilot_msgs__msg__Rc__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0xef, 0x9f, 0x1f, 0x1b, 0x0e, 0xcb, 0x72,
      0xa7, 0x8d, 0x71, 0x90, 0x15, 0x15, 0x27, 0xc0,
      0xb0, 0xd5, 0xee, 0x02, 0xfb, 0x0f, 0x1e, 0x12,
      0xc6, 0xac, 0x18, 0x74, 0xa3, 0x35, 0x58, 0xcc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char ardupilot_msgs__msg__Rc__TYPE_NAME[] = "ardupilot_msgs/msg/Rc";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char ardupilot_msgs__msg__Rc__FIELD_NAME__header[] = "header";
static char ardupilot_msgs__msg__Rc__FIELD_NAME__is_connected[] = "is_connected";
static char ardupilot_msgs__msg__Rc__FIELD_NAME__receiver_rssi[] = "receiver_rssi";
static char ardupilot_msgs__msg__Rc__FIELD_NAME__channels[] = "channels";
static char ardupilot_msgs__msg__Rc__FIELD_NAME__active_overrides[] = "active_overrides";

static rosidl_runtime_c__type_description__Field ardupilot_msgs__msg__Rc__FIELDS[] = {
  {
    {ardupilot_msgs__msg__Rc__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Rc__FIELD_NAME__is_connected, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Rc__FIELD_NAME__receiver_rssi, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Rc__FIELD_NAME__channels, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_BOUNDED_SEQUENCE,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Rc__FIELD_NAME__active_overrides, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ardupilot_msgs__msg__Rc__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ardupilot_msgs__msg__Rc__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ardupilot_msgs__msg__Rc__TYPE_NAME, 21, 21},
      {ardupilot_msgs__msg__Rc__FIELDS, 5, 5},
    },
    {ardupilot_msgs__msg__Rc__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# returns true if radio is connected.\n"
  "bool is_connected\n"
  "\n"
  "# returns [0, 100] for receiver RSSI.\n"
  "uint8 receiver_rssi\n"
  "\n"
  "# channels values.\n"
  "int16[<=32] channels\n"
  "\n"
  "# sets true if a channel is overridden.\n"
  "bool[<=32] active_overrides";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ardupilot_msgs__msg__Rc__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ardupilot_msgs__msg__Rc__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 249, 249},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ardupilot_msgs__msg__Rc__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ardupilot_msgs__msg__Rc__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
