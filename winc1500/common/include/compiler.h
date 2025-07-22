// File: winc1500/common/include/compiler.h
#ifndef COMPILER_H
#define COMPILER_H

/* Minimal stub to satisfy WINC1500 v19.7.x host driver on ARM GCC */

#include "cmsis_compiler.h"    /* CMSIS: __INLINE, __ASM, __WEAK, etc. */
#include <stddef.h>
#include <stdbool.h>
/* Ensure __INLINE is defined for driver code */
#ifndef __INLINE
#define __INLINE static inline
#endif

/* Some drivers use __attribute__((packed)), but CMSIS_COMPILER already
   defines pack macros if needed—fallback if not present: */
#ifndef __PACKED__
#define __PACKED__ __attribute__((packed))
#endif

#endif /* COMPILER_H */
