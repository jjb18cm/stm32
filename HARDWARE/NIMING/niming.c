#include "niming.h"
#include "main.h"
#include "usart.h"
uint8_t data_to_send[100];

//é€šè¿‡F1å¸§å‘é€4ä¸ªuint16ç±»å‹çš„æ•°æ®
void ANO_DT_Send_F1(uint16_t _a, uint16_t _b, uint16_t _c, uint16_t _d)
{
    uint8_t _cnt = 0;		//è®¡æ•°å€¼
    uint8_t sumcheck = 0;  //å’Œæ ¡éªŒ
    uint8_t addcheck = 0; //é™„åŠ å’Œæ ¡éªŒ
    uint8_t i = 0;
	data_to_send[_cnt++] = 0xAA;//å¸§å¤´
    data_to_send[_cnt++] = 0xFF;//ç›®æ ‡åœ°å€
    data_to_send[_cnt++] = 0xF1;//åŠŸèƒ½ç 
    data_to_send[_cnt++] = 8; //æ•°æ®é•¿åº¦
//å•ç‰‡æœºä¸ºå°ç«¯æ¨¡å¼-ä½åœ°å€å­˜æ”¾ä½ä½æ•°æ®ï¼ŒåŒ¿åä¸Šä½æœºè¦æ±‚å…ˆå‘ä½ä½æ•°æ®ï¼Œæ‰€ä»¥å…ˆå‘ä½åœ°å€data_to_send[_cnt++]=BYTE0(_a);Â Â Â 
	data_to_send[_cnt++] = BYTE0(_a);       
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	 for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];//ºÍĞ£Ñé
        addcheck += sumcheck;//¸½¼ÓĞ£Ñé
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//ÕâÀïÊÇ´®¿Ú·¢ËÍº¯Êı
}
//£¬Í¨¹ıF2Ö¡·¢ËÍ4¸öint16ÀàĞÍµÄÊı¾İ
void ANO_DT_Send_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d)   //F2Ö¡  4¸ö  int16 ²ÎÊı
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //ºÍĞ£Ñé
    uint8_t addcheck = 0; //¸½¼ÓºÍĞ£Ñé
    uint8_t i=0;
   data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF2;
    data_to_send[_cnt++] = 8; //Êı¾İ³¤¶È
	//µ¥Æ¬»úÎªĞ¡¶ËÄ£Ê½-µÍµØÖ·´æ·ÅµÍÎ»Êı¾İ£¬ÄäÃûÉÏÎ»»úÒªÇóÏÈ·¢µÍÎ»Êı¾İ£¬ËùÒÔÏÈ·¢µÍµØÖ·
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//ÕâÀïÊÇ´®¿Ú·¢ËÍº¯Êı
}
//Í¨¹ıF3Ö¡·¢ËÍ2¸öint16ÀàĞÍºÍ1¸öint32ÀàĞÍµÄÊı¾İ
void ANO_DT_Send_F3(int16_t _a, int16_t _b, int32_t _c )   //F3Ö¡  2¸ö  int16 ²ÎÊı   1¸ö  int32  ²ÎÊı
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //ºÍĞ£Ñé
    uint8_t addcheck = 0; //¸½¼ÓºÍĞ£Ñé
    uint8_t i=0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF3;
    data_to_send[_cnt++] = 8; //Êı¾İ³¤¶È
	//µ¥Æ¬»úÎªĞ¡¶ËÄ£Ê½-µÍµØÖ·´æ·ÅµÍÎ»Êı¾İ£¬ÄäÃûÉÏÎ»»úÒªÇóÏÈ·¢µÍÎ»Êı¾İ£¬ËùÒÔÏÈ·¢µÍµØÖ·
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
    data_to_send[_cnt++] = BYTE2(_c);
    data_to_send[_cnt++] = BYTE3(_c);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;

	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//ÕâÀïÊÇ´®¿Ú·¢ËÍº¯Êı
}


