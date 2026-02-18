// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#include "ardupilot_msgs/msg/detail/global_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
const rosidl_type_hash_t *
ardupilot_msgs__msg__GlobalPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa2, 0x8c, 0x87, 0x50, 0x99, 0x48, 0x49, 0x62,
      0xd7, 0x2e, 0x5b, 0x82, 0x0e, 0xc7, 0x58, 0x88,
      0x4c, 0x7a, 0x70, 0xc8, 0xda, 0xe5, 0x10, 0x04,
      0x44, 0x51, 0x9c, 0x16, 0xda, 0x36, 0xfa, 0x04,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/twist__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Twist__EXPECTED_HASH = {1, {
    0x9c, 0x45, 0xbf, 0x16, 0xfe, 0x09, 0x83, 0xd8,
    0x0e, 0x3c, 0xfe, 0x75, 0x0d, 0x68, 0x35, 0x84,
    0x3d, 0x26, 0x5a, 0x9a, 0x6c, 0x46, 0xbd, 0x2e,
    0x60, 0x9f, 0xcd, 0xdd, 0xe6, 0xfb, 0x8d, 0x2a,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char ardupilot_msgs__msg__GlobalPosition__TYPE_NAME[] = "ardupilot_msgs/msg/GlobalPosition";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Twist__TYPE_NAME[] = "geometry_msgs/msg/Twist";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__header[] = "header";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__coordinate_frame[] = "coordinate_frame";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__type_mask[] = "type_mask";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__latitude[] = "latitude";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__longitude[] = "longitude";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__altitude[] = "altitude";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__velocity[] = "velocity";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__acceleration_or_force[] = "acceleration_or_force";
static char ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field ardupilot_msgs__msg__GlobalPosition__FIELDS[] = {
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__coordinate_frame, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__type_mask, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__latitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__longitude, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__acceleration_or_force, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__GlobalPosition__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ardupilot_msgs__msg__GlobalPosition__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ardupilot_msgs__msg__GlobalPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ardupilot_msgs__msg__GlobalPosition__TYPE_NAME, 33, 33},
      {ardupilot_msgs__msg__GlobalPosition__FIELDS, 9, 9},
    },
    {ardupilot_msgs__msg__GlobalPosition__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Experimental REP-147 Goal Interface\n"
  "# https://ros.org/reps/rep-0147.html#goal-interface\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "uint8 coordinate_frame\n"
  "uint8 FRAME_GLOBAL_INT = 5\n"
  "uint8 FRAME_GLOBAL_REL_ALT = 6\n"
  "uint8 FRAME_GLOBAL_TERRAIN_ALT = 11\n"
  "\n"
  "uint16 type_mask\n"
  "uint16 IGNORE_LATITUDE = 1 # Position ignore flags\n"
  "uint16 IGNORE_LONGITUDE = 2\n"
  "uint16 IGNORE_ALTITUDE = 4\n"
  "uint16 IGNORE_VX = 8  # Velocity vector ignore flags\n"
  "uint16 IGNORE_VY = 16\n"
  "uint16 IGNORE_VZ = 32\n"
  "uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags\n"
  "uint16 IGNORE_AFY = 128\n"
  "uint16 IGNORE_AFZ = 256\n"
  "uint16 FORCE = 512 # Force in af vector flag\n"
  "uint16 IGNORE_YAW = 1024\n"
  "uint16 IGNORE_YAW_RATE = 2048\n"
  "\n"
  "float64 latitude\n"
  "float64 longitude\n"
  "float32 altitude # in meters, AMSL or above terrain\n"
  "geometry_msgs/Twist velocity\n"
  "geometry_msgs/Twist acceleration_or_force\n"
  "float32 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ardupilot_msgs__msg__GlobalPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ardupilot_msgs__msg__GlobalPosition__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 836, 836},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ardupilot_msgs__msg__GlobalPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ardupilot_msgs__msg__GlobalPosition__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
