// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#include "ardupilot_msgs/msg/detail/status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
const rosidl_type_hash_t *
ardupilot_msgs__msg__Status__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0x9c, 0x4b, 0xf3, 0x0b, 0xf9, 0x4d, 0xf9,
      0xa6, 0x0a, 0x99, 0xb9, 0x53, 0x70, 0xcc, 0xbf,
      0x52, 0xa8, 0x8c, 0x79, 0xd6, 0xce, 0x6c, 0x06,
      0x08, 0x41, 0xa0, 0x55, 0x5d, 0x11, 0xa9, 0xe2,
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

static char ardupilot_msgs__msg__Status__TYPE_NAME[] = "ardupilot_msgs/msg/Status";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char ardupilot_msgs__msg__Status__FIELD_NAME__header[] = "header";
static char ardupilot_msgs__msg__Status__FIELD_NAME__vehicle_type[] = "vehicle_type";
static char ardupilot_msgs__msg__Status__FIELD_NAME__armed[] = "armed";
static char ardupilot_msgs__msg__Status__FIELD_NAME__mode[] = "mode";
static char ardupilot_msgs__msg__Status__FIELD_NAME__flying[] = "flying";
static char ardupilot_msgs__msg__Status__FIELD_NAME__external_control[] = "external_control";
static char ardupilot_msgs__msg__Status__FIELD_NAME__failsafe[] = "failsafe";

static rosidl_runtime_c__type_description__Field ardupilot_msgs__msg__Status__FIELDS[] = {
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__vehicle_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__armed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__flying, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__external_control, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ardupilot_msgs__msg__Status__FIELD_NAME__failsafe, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ardupilot_msgs__msg__Status__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
ardupilot_msgs__msg__Status__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ardupilot_msgs__msg__Status__TYPE_NAME, 25, 25},
      {ardupilot_msgs__msg__Status__FIELDS, 7, 7},
    },
    {ardupilot_msgs__msg__Status__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "uint8 APM_ROVER = 1\n"
  "uint8 APM_ARDUCOPTER = 2\n"
  "uint8 APM_ARDUPLANE = 3\n"
  "uint8 APM_ANTENNATRACKER = 4\n"
  "uint8 APM_UNKNOWN = 5\n"
  "uint8 APM_REPLAY = 6\n"
  "uint8 APM_ARDUSUB = 7\n"
  "uint8 APM_IOFIRMWARE = 8\n"
  "uint8 APM_AP_PERIPH = 9\n"
  "uint8 APM_AP_DAL_STANDALONE = 10\n"
  "uint8 APM_AP_BOOTLOADER = 11\n"
  "uint8 APM_BLIMP = 12\n"
  "uint8 APM_HELI = 13\n"
  "uint8 vehicle_type # From AP_Vehicle_Type.h\n"
  "\n"
  "bool armed # true if vehicle is armed.\n"
  "uint8 mode # Vehicle mode, enum depending on vehicle type.\n"
  "bool flying # True if flying/driving/diving/tracking.\n"
  "bool external_control # True is external control is enabled.\n"
  "\n"
  "uint8 FS_RADIO = 21\n"
  "uint8 FS_BATTERY = 22\n"
  "uint8 FS_GCS = 23\n"
  "uint8 FS_EKF = 24\n"
  "uint8[] failsafe # Array containing all active failsafe.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ardupilot_msgs__msg__Status__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ardupilot_msgs__msg__Status__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 733, 733},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ardupilot_msgs__msg__Status__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ardupilot_msgs__msg__Status__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
