#ifndef _FACE_SOLID_H_
#define _FACE_SOLID_H_

#include "morpho_face_finder.h"
#ifdef __cplusplus
extern "C"
{
#endif

int FaceSolid_Init(int height, int width, unsigned char* p_format);
int FaceSolid_Function(unsigned char *src, morpho_FaceRect ** ppDstFaces, int *pDstFaceNum ,int skip, unsigned char* p_format); /*"YUV420_PLANAR" "YUV420_SEMIPLANAR" "YVU420_SEMIPLANAR"*/
int FaceSolid_Finalize();

#ifdef __cplusplus
}
#endif

#endif
