/*********************************************************************************************************************
* TC264 Opensourec Library ¼´£¨TC264 ¿ªÔ´¿â£©ÊÇÒ»¸ö»ùÓÚ¹Ù·½ SDK ½Ó¿ÚµÄµÚÈı·½¿ªÔ´¿â
* Copyright (c) 2022 SEEKFREE Öğ·É¿Æ¼¼
*
* ±¾ÎÄ¼şÊÇ TC264 ¿ªÔ´¿âµÄÒ»²¿·Ö
*
* TC264 ¿ªÔ´¿â ÊÇÃâ·ÑÈí¼ş
* Äú¿ÉÒÔ¸ù¾İ×ÔÓÉÈí¼ş»ù½ğ»á·¢²¼µÄ GPL£¨GNU General Public License£¬¼´ GNUÍ¨ÓÃ¹«¹²Ğí¿ÉÖ¤£©µÄÌõ¿î
* ¼´ GPL µÄµÚ3°æ£¨¼´ GPL3.0£©»ò£¨ÄúÑ¡ÔñµÄ£©ÈÎºÎºóÀ´µÄ°æ±¾£¬ÖØĞÂ·¢²¼ºÍ/»òĞŞ¸ÄËü
*
* ±¾¿ªÔ´¿âµÄ·¢²¼ÊÇÏ£ÍûËüÄÜ·¢»Ó×÷ÓÃ£¬µ«²¢Î´¶ÔÆä×÷ÈÎºÎµÄ±£Ö¤
* ÉõÖÁÃ»ÓĞÒşº¬µÄÊÊÏúĞÔ»òÊÊºÏÌØ¶¨ÓÃÍ¾µÄ±£Ö¤
* ¸ü¶àÏ¸½ÚÇë²Î¼û GPL
*
* ÄúÓ¦¸ÃÔÚÊÕµ½±¾¿ªÔ´¿âµÄÍ¬Ê±ÊÕµ½Ò»·İ GPL µÄ¸±±¾
* Èç¹ûÃ»ÓĞ£¬Çë²ÎÔÄ<https://www.gnu.org/licenses/>
*
* ¶îÍâ×¢Ã÷£º
* ±¾¿ªÔ´¿âÊ¹ÓÃ GPL3.0 ¿ªÔ´Ğí¿ÉÖ¤Ğ­Òé ÒÔÉÏĞí¿ÉÉêÃ÷ÎªÒëÎÄ°æ±¾
* Ğí¿ÉÉêÃ÷Ó¢ÎÄ°æÔÚ libraries/doc ÎÄ¼ş¼ĞÏÂµÄ GPL3_permission_statement.txt ÎÄ¼şÖĞ
* Ğí¿ÉÖ¤¸±±¾ÔÚ libraries ÎÄ¼ş¼ĞÏÂ ¼´¸ÃÎÄ¼ş¼ĞÏÂµÄ LICENSE ÎÄ¼ş
* »¶Ó­¸÷Î»Ê¹ÓÃ²¢´«²¥±¾³ÌĞò µ«ĞŞ¸ÄÄÚÈİÊ±±ØĞë±£ÁôÖğ·É¿Æ¼¼µÄ°æÈ¨ÉùÃ÷£¨¼´±¾ÉùÃ÷£©
*
* ÎÄ¼şÃû³Æ          zf_device_lora3a22
* ¹«Ë¾Ãû³Æ          ³É¶¼Öğ·É¿Æ¼¼ÓĞÏŞ¹«Ë¾
* °æ±¾ĞÅÏ¢          ²é¿´ libraries/doc ÎÄ¼ş¼ĞÄÚ version ÎÄ¼ş °æ±¾ËµÃ÷
* ¿ª·¢»·¾³          ADS v1.9.4
* ÊÊÓÃÆ½Ì¨          TC264D
* µêÆÌÁ´½Ó          https://seekfree.taobao.com/
*
* ĞŞ¸Ä¼ÇÂ¼
* ÈÕÆÚ              ×÷Õß                ±¸×¢
* 2024-03-29       JKS            first version
********************************************************************************************************************/


#ifndef CODE_ZF_DEVICE_LORA3A22_H_
#define CODE_ZF_DEVICE_LORA3A22_H_

#include "zf_common_headfile.h"

#define LORA3A22_UART_INDEX            (UART_2)              // ¶¨Òå´®¿ÚÒ£¿ØÆ÷Ê¹ÓÃµÄ´®¿Ú
#define LORA3A22_UART_TX_PIN           (UART2_TX_P10_1)      // Ò£¿ØÆ÷½ÓÊÕ»úµÄRXÒı½Å Á¬½Óµ¥Æ¬»úµÄTXÒı½Å
#define LORA3A22_UART_RX_PIN           (UART2_RX_P10_0)      // Ò£¿ØÆ÷½ÓÊÕ»úµÄTXÒı½Å Á¬½Óµ¥Æ¬»úµÄRXÒı½Å
#define LORA3A22_UART_BAUDRATE         (115200)              // Ö¸¶¨ lora3a22 ´®¿ÚËùÊ¹ÓÃµÄµÄ´®¿Ú²¨ÌØÂÊ

#define LORA3A22_DATA_LEN              ( 12  )               // lora3a22Ö¡³¤
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // Ö¡Í·ĞÅÏ¢



typedef struct
{
    uint8 head;                         // Ö¡Í·
    uint8 sum_check;                    // ºÍĞ£Ñé

    int16 joystick[4];
	// joystick[0]:×ó±ßÒ¡¸Ë×óÓÒÖµ
	// joystick[1]:×ó±ßÒ¡¸ËÉÏÏÂÖµ
	// joystick[2]:ÓÒ±ßÒ¡¸Ë×óÓÒÖµ
	// joystick[3]:ÓÒ±ßÒ¡¸ËÉÏÏÂÖµ

    uint8 key[4];
	// °´ÏÂ1 ËÉ¿ª0
    // key[0]-Ò¡¸Ë×ó±ß
    // key[1]-Ò¡¸ËÓÒ±ß
    // key[2]-²àÏò°´¼ü×ó±ß
    // key[3]-²àÏò°´¼üÓÒ±ß

    uint8 switch_key[4];
    // switch_key[0]-×ó±ß²¦Âë¿ª¹Ø_1
    // switch_key[1]-×ó±ß²¦Âë¿ª¹Ø_2
    // switch_key[2]-ÓÒ±ß²¦Âë¿ª¹Ø_1
    // switch_key[3]-ÓÒ±ß²¦Âë¿ª¹Ø_2

}lora3a22_uart_transfer_dat_struct;

extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22½ÓÊÕÔ­Ê¼Êı¾İ
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // Ò£¿ØÆ÷×´Ì¬(1±íÊ¾Õı³££¬·ñÔò±íÊ¾Ê§¿Ø)
extern uint16  lora3a22_response_time;

// @brief  LoRa3A22 é¥æ§æ¨¡å—åˆå§‹åŒ–ï¼Œé…ç½®ä¸²å£å¹¶æ³¨å†Œæ¥æ”¶ä¸­æ–­
void lora3a22_init(void);

#endif
