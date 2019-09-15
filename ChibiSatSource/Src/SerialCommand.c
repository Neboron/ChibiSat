
//================================================================//
//==                                                            ==//
//==                       SerialCommands                       ==//
//==                                                            ==//
//================================================================//

//INCLUDES:
#include "SerialCommand.h"



//DEFINES:
#define   TX_BUFFER_LEN     128
#define   RX_BUFFER_LEN     128
#define   TRM_CMD_LEN       128
#define   TRM_CMD_PTR_LEN   16
#define   TRM_START_BYTE    '/'
#define   MSG_START_BYTE    '>'



enum MSG_P
{
    WAITING,
    TRM_START_BYTE_RECEIVED,
    MSG_START_BYTE_RECEIVED,
    TRM_CMD_RECEIVED
};



//VARIABLES:
extern Console_t Console;



//PRIVATE VARIABLES:
char serial_str_tx[TX_BUFFER_LEN];
char serial_str_rx[RX_BUFFER_LEN];
uint16_t serial_tx_buf_pos;
uint16_t serial_rx_buf_pos;
uint8_t msg_pos = WAITING;
char *saveptr = NULL;

char trm_cmd_buf[RX_BUFFER_LEN];        //there save terminal commands
uint8_t trm_cmd_ptr[TRM_CMD_PTR_LEN];   //array for pointers of begin & end of commands



//FUNCTIONS:
int ConsoleInit(Console_t *Console, const ConsoleCmd_t *Ops, uint16_t OpsLen)
{
  Console->Ops = Ops;
  Console->OpsLen = OpsLen;
  return 0;
}



void reverse(char s[])
{
    int i, j;
    char c;
 
    for (i = 0, j = strlen(s)-1; i<j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}



uint8_t itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
    return i;
}



uint8_t ftoa(float Value, char* Buffer)
 {
     union
     {
         float f;
     
         struct
         {
             unsigned int    mantissa_lo : 16;
             unsigned int    mantissa_hi : 7;    
             unsigned int     exponent : 8;
             unsigned int     sign : 1;
         }hp;
     } helper;
     
     unsigned long mantissa;
     signed char exponent;
     unsigned int int_part;
     char frac_part[3];
     int i, count = 0;
     
     helper.f = Value;
     mantissa = helper.hp.mantissa_lo;
     mantissa += ((unsigned long) helper.hp.mantissa_hi << 16);
     mantissa += 0x00800000;
     exponent = (signed char) helper.hp.exponent - 127;
     
     if (exponent > 18)
     {
         Buffer[0] = 'I';
         Buffer[1] = 'n';
         Buffer[2] = 'f';
         Buffer[3] = '\0';
         return 0;
     }
     
     if (exponent < -3)
     {
         Buffer[0] = '0';
         Buffer[1] = '\0';
         return 0;
     }
     
     count = 0;
     
     if (helper.hp.sign)
     {
         Buffer[0] = '-';
         count++;
     }
     
     int_part = mantissa >> (23 - exponent);    
     itoa(int_part, &Buffer[count]);
     
     for (i = 0; i < 8; i++)
         if (Buffer[i] == '\0')
         {
             count = i;
             break;
         }        
  
     if (count > 5)
         return 0;

     Buffer[count++] = '.';

     switch (0x7 & (mantissa  >> (20 - exponent)))
     {
         case 0:
             frac_part[0] = '0';
             frac_part[1] = '0';
             frac_part[2] = '0';
             break;
         case 1:
             frac_part[0] = '1';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 2:
             frac_part[0] = '2';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 3:
             frac_part[0] = '3';
             frac_part[1] = '7';
             frac_part[2] = '5';            
             break;
         case 4:
             frac_part[0] = '5';
             frac_part[1] = '0';
             frac_part[2] = '0';            
             break;
         case 5:
             frac_part[0] = '6';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 6:
             frac_part[0] = '7';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 7:
             frac_part[0] = '8';
             frac_part[1] = '7';
             frac_part[2] = '5';                    
             break;
     }

     for (i = 0; i < 3; i++)
         if (count < 7)
             Buffer[count++] = frac_part[i];

     Buffer[count] = '\0';
     return count;
 }



void PrintString(char Buf[])
{
    uint32_t len = strlen(Buf);
    SerialBufferSend((uint8_t*)Buf, &len);
}



void PrintInt(int val)
{
    char buf[10];
    uint8_t size = itoa(val, buf);
    PrintString(buf);
}



void PrintFloat(float val)
{
    char buf[10];
    uint8_t size = ftoa(val, buf);
    PrintString(buf);
}



// Must execute in loop or by hendler to avoid TX buffer overflow
void SerialSendHandlar(void)
{
    if(CDC_Transmit_FS((uint8_t*)serial_str_tx,\
        serial_tx_buf_pos) == USBD_OK) serial_tx_buf_pos = 0;
}



void SerialBufferSend(uint8_t* Buf, uint32_t *Len)
{
    for(uint16_t cl = 0; cl <= *Len; cl++)
    {
        serial_str_tx[serial_tx_buf_pos] = Buf[cl];
        serial_tx_buf_pos++;
        if(serial_tx_buf_pos >= TX_BUFFER_LEN) serial_tx_buf_pos = 0;
    }
}



static uint16_t cmdGetArgNum(const char *cmd, uint16_t *Len)
{
    uint16_t i, argc;

    if (!cmd) return 0;

    argc = 0;

    for (i = 0; i < *Len; i++)
        if (cmd[i] == ' ')
        {
            while (cmd[i] == ' ') { i++; }
            if (cmd[i] != '\n' &&
                cmd[i] != '\0' &&
                cmd[i] != '\r') argc++;
        }

    return argc + 1;
}



uint8_t ParseCMD(char* Buf, uint16_t *Len)
{
    char *pch;
    char **argv;
	int i, argc;

    argc = cmdGetArgNum(Buf, Len);
    argv = malloc(argc * sizeof(char *));

    if (!argv) return -1;
    
    i = 0;
    pch = strtok(Buf, " ");
    
    while (pch != NULL)
    {
        argv[i++] = pch;
        pch = strtok (NULL, " ");
    }
    
    for (i = 0; i < Console.OpsLen; i++)
    {
        if (!strncmp(argv[0], Console.Ops[i].cmd, strlen(Console.Ops[i].cmd)))
        {
            if (!Console.Ops[i].doit(argc, argv))
            {

            }
            else
            {
                /** TODO: error handling */
            }
            break;
        }
    }
    free(argv);
    return 0;
    
    //PrintInt(argc);
    //SerialSendHandlar();
}



// Called from CDC_Receive_FS in "usbd_cdc_if.h" (Inerrupt handler)
void SerialParse(uint8_t* Buf, uint32_t *Len)
{
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

    for(uint32_t cl = 0; cl <= *Len; cl++)
    {
        switch (msg_pos)
        {
            case WAITING:
                if(Buf[cl] == MSG_START_BYTE)      msg_pos = MSG_START_BYTE_RECEIVED;
                else if(Buf[cl] == TRM_START_BYTE)
                {
                    //serial_str_rx[serial_rx_buf_pos] = Buf[cl];
                    //serial_rx_buf_pos++;
                    msg_pos = TRM_START_BYTE_RECEIVED;
                }
                else
                {

                }
                break;

            case TRM_START_BYTE_RECEIVED:
                serial_str_rx[serial_rx_buf_pos] = Buf[cl];
                if(Buf[cl] == '\n')
                {
                    serial_str_rx[serial_rx_buf_pos]   = ' ';
                    serial_str_rx[serial_rx_buf_pos+1] = '\n';
                    msg_pos = TRM_CMD_RECEIVED;
                }
                else if(serial_rx_buf_pos >= RX_BUFFER_LEN)
                {
                    msg_pos = WAITING;
                    serial_rx_buf_pos = 0;
                }
                else serial_rx_buf_pos++;
                break;

            case TRM_CMD_RECEIVED:
                ParseCMD(serial_str_rx, &serial_rx_buf_pos);
                serial_rx_buf_pos = 0;
                msg_pos = WAITING;
                break;
        }
    }
}

