/* xcb_summation — n-ary signed sum (BLOCK_SPECS/summation.md).
 *
 * Reference: modules/scicos_blocks/src/c/summation.c (flag==1 body).
 * nin==1 is the full-reduction mode: y[0] = sum of ALL n input elements,
 * sign ignored. nin>=2 sums elementwise, per-element accumulation over
 * inputs in port order starting from 0 (order is normative).
 * signs entries are +1/-1 (generator-validated); the reference treats <=0
 * as subtract.
 */
#ifndef XCB_SUMMATION_H
#define XCB_SUMMATION_H

#include "xcos2c_types.h"

typedef struct {
    int nin;
    int n;              /* elements per input (nin>=2) / total elements (nin==1) */
    const int *signs;   /* length nin */
} xcb_summation_params_t;

void xcb_summation_step(const xcb_summation_params_t *p,
                        const real_t *const *u, real_t *const *y);

#endif /* XCB_SUMMATION_H */
