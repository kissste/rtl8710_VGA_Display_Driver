// 
//  Font data for Liberation Mono 20pt
// 

#include <stdint.h>
#include <stdbool.h>
#include "font.h"
#include "user/atcmd_user.h"

#define bit_test(D,i) (D & (0x01 << i))

uint8_t TDFPutChar(uint16_t x, uint16_t y, char c, FONT_INFO *TDFFont, uint8_t ForColor, uint8_t BackColor) //returns char width or 0 
{ 
   //DBG_8195A("TDFPutChar started %d, %d, %c\n",x,y,c);
   FONT_CHAR_INFO *CharInfo; 
   uint8_t *RowPointer; 
   uint8_t CharIndex, row, col, MyCharWidth, dots, col_in_byte = 0; 

   //Space
   if (c == ' ') {
	   MyCharWidth = 6;
	   for (row=0; row<TDFFont->CharacterHeight; row++) 
		{ 
			for (col=0; col<MyCharWidth; col++) 
			{
				pixel(x+col, y, BackColor); 
			}
			y++;
		}
	   return MyCharWidth+2;
   }
   //Other Characters
   if ((c < TDFFont->StartCharacter) || (c > TDFFont->EndCharacter)) 
      return 0; 

   CharIndex = c - TDFFont->StartCharacter;  //Character index 
   //DBG_8195A("CharIndex: %d\n",CharIndex);
   CharInfo = TDFFont->Descriptors;          //Point to start of descriptors 
   //DBG_8195A("0CharWidth: %d\n",CharInfo->Charwidth);
   //DBG_8195A("0CharOffset: %d\n",CharInfo->Offset);
   CharInfo += (uint32_t)CharIndex;          //Point to current char info 

   RowPointer = TDFFont->Bitmaps;            //Point to start of bitmaps 
   //DBG_8195A("RowPointer: 0x%X\n",RowPointer);
   RowPointer += CharInfo->Offset;           //Point to start of c bitmap 

   MyCharWidth = CharInfo->Charwidth;        //Store Char width (variable) 
   //DBG_8195A("CharWidth: %d\n",CharInfo->Charwidth);
   //DBG_8195A("CharOffset: %d\n",CharInfo->Offset);
   //DBG_8195A("pre-loop %d, %d\n",TDFFont->CharacterHeight, MyCharWidth);
   
   for (row=0; row<TDFFont->CharacterHeight; row++) 
   { 
      dots = *(RowPointer++); 
	  //DBG_8195A("\n%c,%d,0x%X:",c,row,dots);
      col_in_byte = 0; 
      for (col=0; col<MyCharWidth; col++) 
      { 
		 //DBG_8195A("%d,%d,%d\n",row,col,col_in_byte);
         if (++col_in_byte > 8)     //end of byte bits? 
         { 
            dots = *(RowPointer++); //read next byte 
            col_in_byte = 1;        //reset 
         } 
         if (bit_test(dots, 7))   //forcolor 
         { 
            pixel(x+col, y, ForColor); 
         } 
         else                    //backcolor 
         { 
            pixel(x+col, y, BackColor); 
         } 
         dots <<= 1; 
      } 
      y++;     //next row 
   } 

   return MyCharWidth+2; 
} 
/////////////////////////////////////////////////////////////////////////////// 
uint8_t TDFPutStr(uint16_t x, uint16_t y, char *Str, uint8_t ForColor, uint8_t BackColor) 
{ 
   //DBG_8195A("TDFPutStr started\n");
   while (*Str) 
   { 
      x += (TDFPutChar(x,y,*Str, &liberationMono_20ptFontInfo,ForColor,BackColor) + 1); 
      Str++; 
   } 
    
   return 1; 
} 
/////////////////////////////////////////////////////////////////////////////// 