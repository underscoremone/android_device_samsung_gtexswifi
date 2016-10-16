/**
 * @file   morpho_face_finder.h
 * @brief  画像内から顔部分の情報取得
 *
 * Copyright (C) 2007-2010 Morpho, Inc.
 */

#ifndef MORPHO_FACE_FINDER_H
#define MORPHO_FACE_FINDER_H

#include "morpho_api.h"
#include "morpho_error.h"
#include "morpho_image_data.h"
#include "morpho_rect_int.h"
#include "morpho_point_int.h"

#define MORPHO_FACE_SOLID_VER "FaceSolid Ver.4.1.3.ZTE0 2012/03/21"

#ifdef __cplusplus
extern "C" {
#endif

enum {
/**<   0° 顔が上向きの状態、眼が口よりもy座標が小さい */
    MORPHO_FACE_FINDER_INCLINATION_0   = 0x00000001, 
/**< 90°  反時計周り */
    MORPHO_FACE_FINDER_INCLINATION_90  = 0x00000002, 
/**< 180° 反時計周り */
    MORPHO_FACE_FINDER_INCLINATION_180 = 0x00000004, 
/**< 270° 反時計周り */
    MORPHO_FACE_FINDER_INCLINATION_270 = 0x00000008, 
    MORPHO_FACE_FINDER_INCLINATION_NUM = 4,
};

/**
 * 検出された顔の位置情報、輝度情報、ID情報（見つけた順番）
 * を保持するための構造体.
 */
typedef struct{
    int face_id;
    int sx;
    int sy;
    int ex;
    int ey;
    int brightness;
    int angle;
    int smile_level;
    int blink_level;
}morpho_FaceRect;

/**
 * 検出した顔のパーツ情報を保持するための構造体.
 */
typedef struct {
    morpho_PointInt right_pupil; /**< 右瞳孔 */
    morpho_PointInt left_pupil; /**< 左瞳孔 */
    morpho_PointInt right_mouth_corner; /**< 口右端 */
    morpho_PointInt left_mouth_corner; /**< 口左端 */
} morpho_FaceParts;


enum {
    /* 黒目 */
    MORPHO_FACE_REG_RIGHT_EYE_PUPIL = 0,
    MORPHO_FACE_REG_LEFT_EYE_PUPIL,

    /* 口 */
    MORPHO_FACE_REG_RIGHT_MOUTH_CORNER,
    MORPHO_FACE_REG_LEFT_MOUTH_CORNER,

    /* 眉毛：右→左 */
    MORPHO_FACE_REG_OUTER_END_OF_RIGHT_EYE_BROW,
    MORPHO_FACE_REG_INNER_END_OF_RIGHT_EYE_BROW,
    MORPHO_FACE_REG_INNER_END_OF_LEFT_EYE_BROW,
    MORPHO_FACE_REG_OUTER_END_OF_LEFT_EYE_BROW,

    /* 目：右→左 */
    MORPHO_FACE_REG_RIGHT_TEMPLE,
    MORPHO_FACE_REG_OUTER_CORNER_OF_RIGHT_EYE,
    MORPHO_FACE_REG_INNER_CORNER_OF_RIGHT_EYE,
    MORPHO_FACE_REG_INNER_CORNER_OF_LEFT_EYE,
    MORPHO_FACE_REG_OUTER_CORNER_OF_LEFT_EYE,
    MORPHO_FACE_REG_LEFT_TEMPLE,
    
    /* 鼻 */
    MORPHO_FACE_REG_TIP_OF_NOSE,
    MORPHO_FACE_REG_RIGHT_NOSTRIL,
    MORPHO_FACE_REG_LEFT_NOSTRIL,

    /* 口 */
    MORPHO_FACE_REG_CENTER_POINT_ON_OUTER_EDGE_OF_UPPER_LIP,
    MORPHO_FACE_REG_CENTER_POINT_ON_OUTER_EDGE_OF_LOWER_LIP,

    /* 顎 */
    MORPHO_FACE_REG_TIP_OF_CHIN,

    
    /**** EXTRA Face Parts ****/
    MORPHO_FACE_REG_CENTER_POINT_ON_OUTER_EDGE_OF_RIGHT_EYE_BROW,
    MORPHO_FACE_REG_CENTER_POINT_ON_OUTER_EDGE_OF_LEFT_EYE_BROW,
    MORPHO_FACE_REG_CONTER_POINT_ON_UPPER_EDGE_OF_RIGHT_EYELID,
    MORPHO_FACE_REG_CONTER_POINT_ON_LOWER_EDGE_OF_RIGHT_EYELID,
    MORPHO_FACE_REG_CONTER_POINT_ON_UPPER_EDGE_OF_LEFT_EYELID,
    MORPHO_FACE_REG_CONTER_POINT_ON_LOWER_EDGE_OF_LEFT_EYELID,

    MORPHO_FACE_REG_OF_UPPER_POINT_ON_RIGHT_FACE_LINE,
    MORPHO_FACE_REG_OF_LOWER_POINT_ON_RIGHT_FACE_LINE,
    MORPHO_FACE_REG_OF_UPPER_POINT_ON_LEFT_FACE_LINE,
    MORPHO_FACE_REG_OF_LOWER_POINT_ON_LEFT_FACE_LINE,

    MORPHO_FACE_REG_LIP0,
    MORPHO_FACE_REG_LIP1,
    MORPHO_FACE_REG_LIP2,
    MORPHO_FACE_REG_LIP3,
    MORPHO_FACE_REG_LIP4,
    MORPHO_FACE_REG_LIP5,
    MORPHO_FACE_REG_LIP6,
    MORPHO_FACE_REG_LIP7,
    MORPHO_FACE_REG_LIP8,
    MORPHO_FACE_REG_LIP9,
    
    MORPHO_FACE_REG_NUM_FACE_PARTS
};


/**
 * 顔検出器
 */
typedef struct{
    void * p;
}morpho_FaceFinder;


/**
 * 使用するメモリ量を返します.
 * 
 * @param[in] width 入力画像の幅
 * @param[in] height 入力画像の高さ
 * @return 必要なメモリサイズ(byte)
 * 
 */
MORPHO_API(int)
morpho_FaceFinder_getBufferSize(int width , int height);

/**
 * 顔検出器を初期化します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_buffer     ヒープ領域へのポインタ
 * @param[in] buffer_size  使用可能ヒープ領域のサイズ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_initialize( morpho_FaceFinder * p_facefinder , void * p_buffer, int buffer_size);

/**
 * 顔検出器を開始します.
 * これ以降、モード、最大検出人数の選択をすることはできません.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @return エラーコード(morpho_error.h)
 */

MORPHO_API(int)
morpho_FaceFinder_start( morpho_FaceFinder * p_facefinder );

/**
 * 対象画像に対して、顔検出処理を開始します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_image      画像へのポインタ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detectStart( morpho_FaceFinder * p_facefinder, morpho_ImageData * p_image );

/**
 * 対象画像に対して、顔検出処理を実行します.
 *
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_progress      検出処理状況(0-32768)
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detect( morpho_FaceFinder * p_facefinder, int *p_progress);

/**
 * 対象画像に対して、顔検出処理を終了します.
 *
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_image      画像へのポインタ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detectEnd( morpho_FaceFinder * p_facefinder );

/**
 * 検出された顔の矩形情報をリセットします
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detectReset( morpho_FaceFinder * p_facefinder );

/**
 * 検出された顔の情報を取得します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_face_num  検出された人数
 * @param[out] p_faces     検出された顔に関する情報
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getFaces( morpho_FaceFinder * p_facefinder, int * p_face_num, morpho_FaceRect * p_faces );

/**
 * 顔検出器を破棄します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_finalize( morpho_FaceFinder * p_facefinder );

/**
 * 検出する画像のフォーマットを指定します.
 * 現在対応しているフォーマットは以下の通りです。
 * YUV422_YUYV, YUV422_YVYU, YUV422_YYUV, YUV422_YYVU,
 * YUV422_VYUY, YUV422_UYVY, YUV422_VUYY, YUV422_UVYY,
 * YUV422_PLANAR, YUV422_SEMIPLANAR,
 * YUV420_PLANAR, YUV420_SEMIPLANAR,
 * YVU422_SEMIPLANAR, YVU420_SEMIPLANAR,
 * RGB565, RGB888
 * ※以下は、笑顔/パーツ検出は未対応
 * YUV444_YUV, YUV444_YVU, YUV444_UYV,
 * YUV444_VYU, YUV444_UVY, YUV444_VUY
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_format     画像フォーマット
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setImageFormat( morpho_FaceFinder * p_facefinder, const char * p_format );

/**
 * 設定されている画像のフォーマットを取得します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_format     画像フォーマット
 */
MORPHO_API(int)
morpho_FaceFinder_getImageFormat( morpho_FaceFinder * p_facefinder, const char ** p_format );

/**
 * 検出する画像のサイズを指定します.
 * 本指定はmorpho_FaceFinder_getBufferSize()指定時と同様の値を指定してください
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] width     画像の幅
 * @param[in] height    画像の高さ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setImageSize( morpho_FaceFinder * p_facefinder, int width, int height);

/**
 * 検出する画像のサイズを取得します.
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_width     画像の幅
 * @param[out] p_height    画像の高さ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getImageSize( morpho_FaceFinder * p_facefinder, int *p_width, int *p_height);

/**
 * 検出する傾きの対象角度を指定します.
 * 
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[in] inclination_type 検出対象角度のタイプ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setInclinationType( morpho_FaceFinder * p_facefinder, int inclination_type );

/**
 * 設定されている検出する傾きの対象角度を取得します.
 * 
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[in] p_inclination_type 検出対象角度のタイプ
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getInclinationType( morpho_FaceFinder * p_facefinder, int *p_inclination_type );

/**
 * 検出する矩形の最小サイズを指定します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] min_detect_size 最小矩形サイズ幅
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setMinDetectSize( morpho_FaceFinder * p_facefinder, int min_detect_size );

/**
 * 検出する矩形の最小サイズを取得します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_min_detect_size 最小矩形サイズ幅
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getMinDetectSize( morpho_FaceFinder *p_facefinder, int *p_min_detect_size );

/**
 * 検出する矩形の最大サイズを設定します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] max_detect_size 最小矩形サイズ幅
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setMaxDetectSize( morpho_FaceFinder * p_facefinder, int max_detect_size );

/**
 * 検出する矩形の最大サイズを取得します.
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_max_detect_size 最小矩形サイズ幅
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getMaxDetectSize( morpho_FaceFinder *p_facefinder, int *p_max_detect_size );

/**
 * 顔情報取得時、顔情報のソート方法を指定します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] sort_rule    ソート方法
 * 
 * @return エラーコード(morpho_error.h)
 */

enum {
    MORPHO_FACE_FINDER_SORT_RULE_NONE = 0, /**< ソートなし */
    MORPHO_FACE_FINDER_SORT_RULE_ASCEND_FACE_SIZE, /**< 顔矩形サイズが小さい順 */
    MORPHO_FACE_FINDER_SORT_RULE_DESCEND_FACE_SIZE, /**< 顔矩形サイズが小さい順 */
    MORPHO_FACE_FINDER_SORT_RULE_ASCEND_FACE_ID, /**< 顔IDが小さい順 */
    MORPHO_FACE_FINDER_SORT_RULE_DESCEND_FACE_ID, /**< 顔IDが大きい順 */
    MORPHO_FACE_FINDER_SORT_RULE_NUM,
};

MORPHO_API(int)
morpho_FaceFinder_setSortRule( morpho_FaceFinder * p_facefinder, int sort_rule );

/**
 * 顔情報取得時、顔情報のソート方法を取得します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[out] p_sort_rule ソート方法
 * 
 * @return エラーコード(morpho_error.h)
 */

MORPHO_API(int)
morpho_FaceFinder_getSortRule( morpho_FaceFinder * p_facefinder, int *p_sort_rule );

/**
 * 最大検出人数を指定します
 * 
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[in] max_face_num 最大検出人数（設定可能範囲： 1～MORPHO_FACE_FINDER_MAX_NUM_FACE）
 * 
 * @return エラーコード(morpho_error.h)
 */
enum {
    MORPHO_FACE_FINDER_MAX_FACE_NUM = 20,
};

MORPHO_API(int)
morpho_FaceFinder_setMaxFaceNum( morpho_FaceFinder *p_facefinder, int max_face_num );

/**
 * 指定されている最大検出人数を取得します
 * 
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[out] p_max_face_num 最大検出人数
 * 
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getMaxFaceNum( morpho_FaceFinder *p_facefinder, int *p_max_face_num );

/**
 * （評価用）
 * 探索範囲の矩形を入力画像の座標で設定します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_search_rect 検索範囲の矩形（入力画像内の座標を指定）
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setSearchAreaRect( morpho_FaceFinder * p_facefinder, morpho_RectInt *p_search_rect);

/**
 * （評価用）
 * 設定されている探索範囲の矩形を取得します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_search_rect 検索範囲の矩形（入力画像内の座標を指定）
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getSearchAreaRect( morpho_FaceFinder * p_facefinder, morpho_RectInt *p_search_rect);

/**
 * 顔周辺検索カウントの指定
 * 
 * 検出された顔周辺8方向を1Pixelずらして検索し,指定数以上の顔の検出がなければ誤検出とみなす
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[in] false_suppression_level 周辺検索カウント（設定可能範囲： 0～8）
 * 
 * デフォルト:1
 * 0:処理を行わない
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setFalseDetectionSuppressionLevel( morpho_FaceFinder * p_facefinder, int false_suppression_level );

/**
 * 顔周辺検索カウントの取得
 * 
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[out] p_false_suppression_level 周辺検索カウント
 * 
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getFalseDetectionSuppressionLevel( morpho_FaceFinder * p_facefinder, int *p_false_suppression_level );

/**
 * TrackValidateStepの指定
 * トラッキング中に再検索を実施する頻度
 * morpho_FaceFinder_detectStartが設定値回数呼び出される間は、簡易的な顔検出が実行される
 *
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[in] light_track_num （設定可能範囲： 0～30）
 * 
 * デフォルト:30
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_setLightTrackNum( morpho_FaceFinder * p_facefinder, int light_track_num );

/**
 * TrackValidateStepの取得
 * 
 *
 * @param[in] p_facefinder  顔検出器へのポインタ
 * @param[out] p_light_track_num
 * 
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_getLightTrackNum( morpho_FaceFinder * p_facefinder, int *p_light_track_num );

/**
 * 指定した顔IDのパーツ情報を取得します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_image      入力画像
 * @param[in] p_rect       顔情報
 * @param[out] p_parts     顔パーツ情報
 * 
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detectFaceParts( morpho_FaceFinder *p_facefinder, morpho_ImageData *p_image, morpho_FaceRect *p_rect, morpho_FaceParts *p_parts );

/**
    * 指定した顔IDのパーツ情報(詳細)を取得します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] p_image      入力画像
 * @param[in] p_rect       顔情報
 * @param[out] p_points    顔パーツ座標
 * @param[in]  point_num   座標(p_points)配列数
 * 
 * @return エラーコード(morpho_error.h)
 */
MORPHO_API(int)
morpho_FaceFinder_detectFacePartsDetail( morpho_FaceFinder *p_facefinder, morpho_ImageData *p_image, morpho_FaceRect *p_rect, morpho_PointInt *p_points, int point_num );

/**
 * 検出された顔の笑顔度を判定します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] image        入力画像へのポインタ
 * @param[in] p_face       検出された顔に関する情報(本APIのみ顔情報は一つだけ指定)
 * @param[out] p_smile     笑顔度(0～255)
 * @return エラーコード(morpho_error.h)
 */
 
MORPHO_API(int)
morpho_FaceFinder_detectSmile( morpho_FaceFinder * p_facefinder, morpho_ImageData * p_image, morpho_FaceRect *p_face, int* p_smile);

/**
 * 検出された顔の瞬き度を判定します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] image        入力画像へのポインタ
 * @param[in] p_face       検出された顔に関する情報(本APIのみ顔情報は一つだけ指定)
 * @param[out] p_blink     笑顔度(0～255)
 * @return エラーコード(morpho_error.h)
 */

MORPHO_API(int)
morpho_FaceFinder_detectBlink( morpho_FaceFinder * p_facefinder, morpho_ImageData * p_image, morpho_FaceRect *p_face, int* p_blink);


/**
 * 指定区域内で顔が検出されているか判定します
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in] image        入力画像へのポインタ
 * @param[in] p_face_num  検出された人数
 * @param[in] p_face       検出された顔に関する情報
 * @param[in] shotarea     撮影検出範囲（指定画像の中心から何％を検出範囲とするか）
 * @param[out] p_out_bool   検出結果(1：指定範囲内、0：指定範囲外)
 * 
 * @return エラーコード(morpho_error.h)
 */

 
MORPHO_API(int)
morpho_FaceFinder_isShutterChance( morpho_FaceFinder * p_facefinder, morpho_ImageData * p_image, int face_num, morpho_FaceRect * p_faces,int shotarea, int* p_out_bool);

/**
 * 顔の輝度が基準値（levelで決まる）以下の場合明るさを補正します
 * デフォルトでは使用不可の関数です。
 * 
 * @param[in] p_facefinder 顔検出器へのポインタ
 * @param[in/out] p_image  入出力画像へのポインタ
 * @param[in] p_face_num   検出された人数
 * @param[in] p_faces      検出された顔に関する情報
 * @param[in] level        逆光補正レベル(0-255)
 * @return エラーコード(morpho_error.h)
 */

MORPHO_API(int)
morpho_FaceFinder_normalizeColor( morpho_FaceFinder * p_facefinder, morpho_ImageData * p_image, int face_num, morpho_FaceRect * p_faces, int level );

/**
 * 本ライブラリのバージョン情報を取得します
 * 
 * @return [out]const char* p_Version:本ライブラリのバージョン情報
 */
MORPHO_API(const char*)
morpho_FaceFinder_getVersion( void );


#ifdef __cplusplus
}
#endif

#endif /* MORPHO_FACE_FINDER_H */

