#ifndef WRITEMAT_H
#define WRITEMAT_H

#endif // WRITEMAT_H

#ifndef __OPENCV_ALL_HPP__
#include <opencv2/opencv.hpp>
#endif
#ifndef	_STDLIB_H
#include <stdlib.h>
#endif
#ifndef _STDIO_H
#include <stdio.h>
#endif
#ifndef _GCC_WRAP_STDINT_H
#include <stdint.h>
#endif

using namespace cv;

void writeMat( Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true );
