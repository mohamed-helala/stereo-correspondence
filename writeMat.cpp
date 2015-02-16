/* Author: Philip G. Lee <rocketman768@gmail.com>
 * I, Philip G. Lee, hereby disclaim any Copyright that I might hold
 * by default.
 */
#include <writeMat.h>

/*!
 *  \author Philip G. Lee <rocketman768@gmail.com>
 *  Write \b mat into \b filename
 *  in uncompressed .mat format (Level 5 MATLAB) for Matlab.
 *  The variable name in matlab will be \b varName. If
 *  \b bgr2rgb is true and there are 3 channels, swaps 1st and 3rd
 *  channels in the output. This is needed because OpenCV matrices
 *  are bgr, while Matlab is rgb. This has been tested to work with
 *  3-channel single-precision floating point matrices, and I hope
 *  it works on other types/channels, but not exactly sure.
 *  Documentation at <http://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf>
 */

void writeMat( Mat const& mat, const char* filename, const char* varName, bool bgr2rgb )
{
   int textLen = 116;
   char* text;
   int subsysOffsetLen = 8;
   char* subsysOffset;
   int verLen = 2;
   char* ver;
   char flags;
   int bytes;
   int padBytes;
   int bytesPerElement;
   int i,j,k,k2;
   bool doBgrSwap;
   char mxClass;
   int32_t miClass;
   uchar const* rowPtr;
   uint32_t tmp32;
   float tmp;
   FILE* fp;

   // Matlab constants.
   const uint16_t MI = 0x4d49; // Contains "MI" in ascii.
   const int32_t miINT8 = 1;
   const int32_t miUINT8 = 2;
   const int32_t miINT16 = 3;
   const int32_t miUINT16 = 4;
   const int32_t miINT32 = 5;
   const int32_t miUINT32 = 6;
   const int32_t miSINGLE = 7;
   const int32_t miDOUBLE = 9;
   const int32_t miMATRIX = 14;
   const char mxDOUBLE_CLASS = 6;
   const char mxSINGLE_CLASS = 7;
   const char mxINT8_CLASS = 8;
   const char mxUINT8_CLASS = 9;
   const char mxINT16_CLASS = 10;
   const char mxUINT16_CLASS = 11;
   const char mxINT32_CLASS = 12;
   const char mxUINT32_CLASS = 13;
   const uint64_t zero = 0; // Used for padding.

   fp = fopen( filename, "wb" );

   if( fp == 0 )
      return;

   const int rows = mat.rows;
   const int cols = mat.cols;
   const int chans = mat.channels();

   doBgrSwap = (chans==3) && bgr2rgb;

   // I hope this mapping is right :-/
   switch( mat.depth() )
   {
   case CV_8U:
      mxClass = mxUINT8_CLASS;
      miClass = miUINT8;
      bytesPerElement = 1;
      break;
   case CV_8S:
      mxClass = mxINT8_CLASS;
      miClass = miINT8;
      bytesPerElement = 1;
      break;
   case CV_16U:
      mxClass = mxUINT16_CLASS;
      miClass = miUINT16;
      bytesPerElement = 2;
      break;
   case CV_16S:
      mxClass = mxINT16_CLASS;
      miClass = miINT16;
      bytesPerElement = 2;
      break;
   case CV_32S:
      mxClass = mxINT32_CLASS;
      miClass = miINT32;
      bytesPerElement = 4;
      break;
   case CV_32F:
      mxClass = mxSINGLE_CLASS;
      miClass = miSINGLE;
      bytesPerElement = 4;
      break;
   case CV_64F:
      mxClass = mxDOUBLE_CLASS;
      miClass = miDOUBLE;
      bytesPerElement = 8;
      break;
   default:
      return;
   }

   //==================Mat-file header (128 bytes, page 1-5)==================
   text = new char[textLen]; // Human-readable text.
   memset( text, ' ', textLen );
   text[textLen-1] = '\0';
   const char* t = "MATLAB 5.0 MAT-file, Platform: PCWIN";
   memcpy( text, t, strlen(t) );

   subsysOffset = new char[subsysOffsetLen]; // Zeros for us.
   memset( subsysOffset, 0x00, subsysOffsetLen );
   ver = new char[verLen];
   ver[0] = 0x00;
   ver[1] = 0x01;

   fwrite( text, 1, textLen, fp );
   fwrite( subsysOffset, 1, subsysOffsetLen, fp );
   fwrite( ver, 1, verLen, fp );
   // Endian indicator. MI will show up as "MI" on big-endian
   // systems and "IM" on little-endian systems.
   fwrite( &MI, 2, 1, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //===================Data element tag (8 bytes, page 1-8)==================
   bytes = 16 + 24 + (8 + strlen(varName) + (8-(strlen(varName)%8))%8)
      + (8 + rows*cols*chans*bytesPerElement);
   fwrite( &miMATRIX, 4, 1, fp ); // Data type.
   fwrite( &bytes, 4, 1, fp); // Data size in bytes.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //====================Array flags (16 bytes, page 1-15)====================
   bytes = 8;
   fwrite( &miUINT32, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );
   flags = 0x00; // Complex, logical, and global flags all off.

   tmp32 = 0;
   tmp32 = (flags << 8 ) | (mxClass);
   fwrite( &tmp32, 4, 1, fp );

   fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //===============Dimensions subelement (24 bytes, page 1-17)===============
   bytes = 12;
   fwrite( &miINT32, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );

   fwrite( &rows, 4, 1, fp );
   fwrite( &cols, 4, 1, fp );
   fwrite( &chans, 4, 1, fp );
   fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //==Array name (8 + strlen(varName) + (8-(strlen(varName)%8))%8 bytes, page 1-17)==
   bytes = strlen(varName);

   fwrite( &miINT8, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );
   fwrite( varName, 1, bytes, fp );

   // Pad to nearest 64-bit boundary.
   padBytes = (8-(bytes%8))%8;
   fwrite( &zero, 1, padBytes, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //====Matrix data (rows*cols*chans*bytesPerElement+8 bytes, page 1-20)=====
   bytes = rows*cols*chans*bytesPerElement;
   fwrite( &miClass, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );

   for( k = 0; k < chans; ++k )
   {
      if( doBgrSwap )
      {
         k2 = (k==0)? 2 : ((k==2)? 0 : 1);
      }
      else
         k2 = k;

      for( j = 0; j < cols; ++j )
      {
         for( i = 0; i < rows; ++i )
         {
            rowPtr = mat.data + mat.step*i;
            fwrite( rowPtr + (chans*j + k2)*bytesPerElement, bytesPerElement, 1, fp );
         }
      }
   }

   // Pad to 64-bit boundary.
   padBytes = (8-(bytes%8))%8;
   fwrite( &zero, 1, padBytes, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   fclose(fp);
   delete[] text;
   delete[] subsysOffset;
   delete[] ver;
}
