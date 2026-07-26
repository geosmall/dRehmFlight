/* xcos2c_types.h — numeric policy for generated code and the block runtime.
 *
 * CODEGEN_SPEC §3: real_t is float by default (ARM target, dRehmFlight
 * convention); defining XCOS2C_REAL64 selects double (host golden builds).
 * Conformance builds use -O2 -ffp-contract=off -fno-math-errno; never
 * -ffast-math (flags are part of the codegen contract).
 */
#ifndef XCOS2C_TYPES_H
#define XCOS2C_TYPES_H

#include <stdint.h>

#ifdef XCOS2C_REAL64
typedef double real_t;
#else
typedef float real_t;
#endif

#endif /* XCOS2C_TYPES_H */
