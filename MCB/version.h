/*
 * ============================================================================
 * version.h - Single-sourced firmware version for the MCB
 * ============================================================================
 *
 * Every version representation (serial banner, #G parameter, #?EF?# identify,
 * boot log) is derived from the macros below. Do not hardcode the version
 * string anywhere else. See ../VERSIONING.md for the suite versioning policy.
 */

#ifndef MCB_VERSION_H
#define MCB_VERSION_H

#define FIRMWARE_VERSION_MAJOR   2
#define FIRMWARE_VERSION_MINOR   1
#define FIRMWARE_VERSION_PATCH   0

// Human-readable semantic version string (authoritative).
#define FIRMWARE_VERSION         "2.1.0"

// Integer version for the #?EF?# identify response:
//   MAJOR*10000 + MINOR*100 + PATCH  ->  2.1.0 == 20100
#define FIRMWARE_VERSION_INT     20100

// 2.1.x release codename (cosmetic; see ../VERSIONING.md).
#define FIRMWARE_CODENAME        "Orbit"

#endif // MCB_VERSION_H
