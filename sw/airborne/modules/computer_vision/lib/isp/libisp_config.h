#ifndef _LIBISP_CONFIG_H
#define _LIBISP_CONFIG_H

#define ISP_CFA 2
#include "boards/bebop.h"

#define CAMERA_H_FISHEYE_RADIUS 1920
#define CAMERA_H_FISHEYE_CENTER_X ((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN)/2)
#define CAMERA_H_FISHEYE_CENTER_Y ((CFG_MT9F002_Y_ADDR_MAX - CFG_MT9F002_Y_ADDR_MIN)/2)

#define COMPLEMENT_2(i, r) (((i) >= 0) ? (r) : (~(r) + 1) & 0x3fff)
#define Q311(i) (COMPLEMENT_2(i, (unsigned)(((ABS(i)) * (1 << 11)) + 0.5)))
#define AVI_CONV_MATRIX(_c00, _c01, _c02,                                    \
                        _c10, _c11, _c12,                                    \
                        _c20, _c21, _c22)                                    \
        .coeff_01_00 = {{ .coeff_00 = Q311(_c00), .coeff_01 = Q311(_c01) }}, \
        .coeff_10_02 = {{ .coeff_02 = Q311(_c02), .coeff_10 = Q311(_c10) }}, \
        .coeff_12_11 = {{ .coeff_11 = Q311(_c11), .coeff_12 = Q311(_c12) }}, \
        .coeff_21_20 = {{ .coeff_20 = Q311(_c20), .coeff_21 = Q311(_c21) }}, \
        .coeff_22    = {{ .coeff_22 = Q311(_c22) }}

#define AVI_CONV_OFFSETS(_ryin, _ryout,                                 \
                         _guin, _guout,                                 \
                         _bvin, _bvout)                                 \
        .offset_ry = {{ .offset_in = _ryin, .offset_out = _ryout }},    \
        .offset_gu = {{ .offset_in = _guin, .offset_out = _guout }},    \
        .offset_bv = {{ .offset_in = _bvin, .offset_out = _bvout }}

#define AVI_CONV_CLIPS(_rymin, _rymax,                                  \
                       _gumin, _gumax,                                  \
                       _bvmin, _bvmax)                                  \
        .clip_ry = {{ .clip_min = _rymin, .clip_max = _rymax }},        \
        .clip_gu = {{ .clip_min = _gumin, .clip_max = _gumax }},        \
        .clip_bv = {{ .clip_min = _bvmin, .clip_max = _bvmax }}

extern struct libisp_config isp_config;

#endif /* _LIBISP_CONFIG_H */
