#include "xcb_summation.h"

void xcb_summation_step(const xcb_summation_params_t *p,
                        const real_t *const *u, real_t *const *y)
{
    if (p->nin == 1) {
        y[0][0] = 0.0;
        for (int j = 0; j < p->n; ++j) {
            y[0][0] = y[0][0] + u[0][j];
        }
    } else {
        for (int j = 0; j < p->n; ++j) {
            y[0][j] = 0.0;
            for (int k = 0; k < p->nin; ++k) {
                if (p->signs[k] > 0) {
                    y[0][j] = y[0][j] + u[k][j];
                } else {
                    y[0][j] = y[0][j] - u[k][j];
                }
            }
        }
    }
}
