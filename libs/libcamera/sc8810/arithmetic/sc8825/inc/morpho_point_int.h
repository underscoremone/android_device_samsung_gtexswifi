/**
 * @file   morpho_point_int.h
 * @brief  座標データ構造体定義
 * @version  1.0.0
 * @date     2008-10-17
 *
 * Copyright (C) 2008 Morpho, Inc.
 */

#ifndef MORPHO_POINT_INT_H
#define MORPHO_POINT_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/** 座標データ */
typedef struct {
    int x; /**< X座標 */
    int y; /**< Y座標 */
} morpho_PointInt;

#ifdef __cplusplus
}
#endif

#endif /* MORPHO_POINT_INT_H */
