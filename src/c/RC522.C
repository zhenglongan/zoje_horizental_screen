#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\variables.h" 
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\delay.h" 
#include "..\..\include\action.h" 
#include "..\..\include\MFRC522.h" 

#define MAXRLEN 18

/////////////////////////////////////////////////////////////////////
//��  �ܣ�Ѱ��
//����˵��: req_code[IN]:Ѱ����ʽ
//        0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�
//        0x26 = Ѱδ��������״̬�Ŀ�
//      pTagType[OUT]����Ƭ���ʹ���
//        0x4400 = Mifare_UltraLight
//        0x0400 = Mifare_One(S50)
//        0x0200 = Mifare_One(S70)
//        0x0800 = Mifare_Pro(X)
//        0x4403 = Mifare_DESFire
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdRequest(UINT8 req_code,UINT8 *pTagType)
{
   INT8   status;  
   UINT16 unLen;
   UINT8 ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);
   SetBitMask(TxControlReg,0x03);
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
   
   if ((status == MI_OK) && (unLen == 0x10))
   {  
     *pTagType   = ucComMF522Buf[0];
     *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ�����ײ
//����˵��: pSnr[OUT]:��Ƭ���кţ�4�ֽ�
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////  
INT8 PcdAnticoll(UINT8 *pSnr)
{
    INT8 status;
    UINT8 i,snr_check=0;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 
  

  ClearBitMask(Status2Reg,0x08);
  WriteRawRC(BitFramingReg,0x00);
  ClearBitMask(CollReg,0x80);
 
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x20;

  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

  if (status == MI_OK)
  {
  	 for (i=0; i<4; i++)
     {   
       *(pSnr+i)  = ucComMF522Buf[i];
       snr_check ^= ucComMF522Buf[i];
     }
     if (snr_check != ucComMF522Buf[i])
     {   status = MI_ERR;  }
  }
  
  SetBitMask(CollReg,0x80);
  return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ�ѡ����Ƭ
//����˵��: pSnr[IN]:��Ƭ���кţ�4�ֽ�
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdSelect(UINT8 *pSnr)
{
    INT8 status;
    UINT8 i;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i=0; i<4; i++)
  {
  	ucComMF522Buf[i+2] = *(pSnr+i);
  	ucComMF522Buf[6]  ^= *(pSnr+i);
  }
  CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
  ClearBitMask(Status2Reg,0x08);

  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
  
  if ((status == MI_OK) && (unLen == 0x18))
  {   status = MI_OK;  }
  else
  {   status = MI_ERR;  }

  return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ���֤��Ƭ����
//����˵��: auth_mode[IN]: ������֤ģʽ
//         0x60 = ��֤A��Կ
//         0x61 = ��֤B��Կ 
//      addr[IN]�����ַ
//      pKey[IN]������
//      pSnr[IN]����Ƭ���кţ�4�ֽ�
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////         
INT8 PcdAuthState(UINT8 auth_mode,UINT8 addr,UINT8 *pKey,UINT8 *pSnr)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = auth_mode;
  ucComMF522Buf[1] = addr;
  for (i=0; i<6; i++)
  {  ucComMF522Buf[i+2] = *(pKey+i);   }
  for (i=0; i<6; i++)
  {  ucComMF522Buf[i+8] = *(pSnr+i);   }
  
  status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
  if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
  {   status = MI_ERR;   }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ���ȡM1��һ������
//����˵��: addr[IN]�����ַ
//      pData[OUT]�����������ݣ�16�ֽ�
//��  ��: �ɹ�����MI_OK
///////////////////////////////////////////////////////////////////// 
INT8 PcdRead(UINT8 addr,UINT8 *pData)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = PICC_READ;
  ucComMF522Buf[1] = addr;
  CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  if ((status == MI_OK) && (unLen == 0x90))
  {
    for (i=0; i<16; i++)
    {  *(pData+i) = ucComMF522Buf[i];   }
  }
  else
  {   status = MI_ERR;   }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ�д���ݵ�M1��һ��
//����˵��: addr[IN]�����ַ
//      pData[IN]��д������ݣ�16�ֽ�
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////          
INT8 PcdWrite(UINT8 addr,UINT8 *pData)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

  if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
  {   status = MI_ERR;   }
    
  if (status == MI_OK)
  {
    for (i=0; i<16; i++)
    {  ucComMF522Buf[i] = *(pData+i);   }
    CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
  }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ����Ƭ��������״̬
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdHalt(void)
{
    INT8 status;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 

  	ucComMF522Buf[0] = PICC_HALT;
  	ucComMF522Buf[1] = 0;
  	CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
  	status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//��MF522����CRC16����
/////////////////////////////////////////////////////////////////////
void CalulateCRC(UINT8 *pIndata,UINT8 len,UINT8 *pOutData)
{
    UINT8 i,n;
  ClearBitMask(DivIrqReg,0x04);
  WriteRawRC(CommandReg,PCD_IDLE);
  SetBitMask(FIFOLevelReg,0x80);
  for (i=0; i<len; i++)
  {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
  WriteRawRC(CommandReg, PCD_CALCCRC);
  i = 0xFF;
  do 
  {
    n = ReadRawRC(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));
  pOutData[0] = ReadRawRC(CRCResultRegL);
  pOutData[1] = ReadRawRC(CRCResultRegM);
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ���λRC522
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdReset(void)
{	
  	WriteRawRC(CommandReg,PCD_RESETPHASE);
	delay_ms(30); 
  	WriteRawRC(ModeReg,0x3D);            //��Mifare��ͨѶ��CRC��ʼֵ0x6363
	delay_ms(30); 
  	WriteRawRC(TReloadRegL,30); 
	delay_ms(30);          
  	WriteRawRC(TReloadRegH,0);
	delay_ms(30); 
  	WriteRawRC(TModeReg,0x8D);
	delay_ms(30); 
  	WriteRawRC(TPrescalerReg,0x3E);
	delay_ms(30); 
  	WriteRawRC(TxAskReg,0x40); 
	delay_ms(30); 
  	ClearBitMask(TestPinEnReg, 0x80);//off MX and DTRQ out
	
 
  	return MI_OK;
}
//////////////////////////////////////////////////////////////////////
//����RC522�Ĺ�����ʽ 
//////////////////////////////////////////////////////////////////////
INT8 M500PcdConfigISOType(UINT8 type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       ClearBitMask(Status2Reg,0x08);

 /*    WriteRawRC(CommandReg,0x20);    //as default   
       WriteRawRC(ComIEnReg,0x80);     //as default
       WriteRawRC(DivlEnReg,0x0);      //as default
	   WriteRawRC(ComIrqReg,0x04);     //as default
	   WriteRawRC(DivIrqReg,0x0);      //as default
	   WriteRawRC(Status2Reg,0x0);//80    //trun off temperature sensor
	   WriteRawRC(WaterLevelReg,0x08); //as default
       WriteRawRC(ControlReg,0x20);    //as default
	   WriteRawRC(CollReg,0x80);    //as default
*/
       WriteRawRC(ModeReg,0x3D);//3F
/*	   WriteRawRC(TxModeReg,0x0);      //as default???
	   WriteRawRC(RxModeReg,0x0);      //as default???
	   WriteRawRC(TxControlReg,0x80);  //as default???
	   WriteRawRC(TxSelReg,0x10);      //as default???
*/
       WriteRawRC(RxSelReg,0x86);//84
 //    WriteRawRC(RxThresholdReg,0x84);//as default
 //    WriteRawRC(DemodReg,0x4D);      //as default

 //    WriteRawRC(ModWidthReg,0x13);//26
       WriteRawRC(RFCfgReg,0x7F);   //4F
/*     WriteRawRC(GsNReg,0x88);        //as default???
	   WriteRawRC(CWGsCfgReg,0x20);    //as default???
       WriteRawRC(ModGsCfgReg,0x20);   //as default???
*/
   	   WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   WriteRawRC(TReloadRegH,0);
       WriteRawRC(TModeReg,0x8D);
	   WriteRawRC(TPrescalerReg,0x3E);
//     PcdSetTmo(106);
       PcdAntennaOn();
   }
   else{ return -1; }
   
   return MI_OK;
}


/////////////////////////////////////////////////////////////////////
//��  �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//      mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
void SetBitMask(UINT8 reg,UINT8 mask)  
{
    INT8 tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//      mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
void ClearBitMask(UINT8 reg,UINT8 mask)  
{
    INT8 tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//��  �ܣ�ͨ��RC522��ISO14443��ͨѶ
//����˵����Command[IN]:RC522������
//      pInData[IN]:ͨ��RC522���͵���Ƭ������
//      InLenByte[IN]:�������ݵ��ֽڳ���
//      pOutData[OUT]:���յ��Ŀ�Ƭ��������
//      *pOutLenBit[OUT]:�������ݵ�λ����
/////////////////////////////////////////////////////////////////////
INT8 PcdComMF522(UINT8 Command, 
                 UINT8 *pInData, 
                 UINT8 InLenByte,
                 UINT8 *pOutData, 
                 UINT16  *pOutLenBit)
{
    INT8 status = MI_ERR;
    UINT8 irqEn   = 0x00;
    UINT8 waitFor = 0x00;
    UINT8 lastBits;
    UINT8 n;
    UINT16 i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    
    for (i=0; i<InLenByte; i++)
    {   
		WriteRawRC(FIFODataReg, pInData[i]);    
	}
    WriteRawRC(CommandReg, Command);
   
    
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }
    
  i = 600;//����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms
// 	i = 2000;
    do 
    {
         n = ReadRawRC(ComIrqReg);
         i--;
		 //delay_ms(1);
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
	      
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   status = MI_ERR;   }
        
   }

   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
   return status;
}


/////////////////////////////////////////////////////////////////////
//��������  
//ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn()
{
    UINT8 i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//�ر�����
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}
/////////////////////////////////////////////////////////////////////
//��  �ܣ��ۿ�ͳ�ֵ
//����˵��: dd_mode[IN]��������
//         0xC0 = �ۿ�
//         0xC1 = ��ֵ
//      addr[IN]��Ǯ����ַ
//      pValue[IN]��4�ֽ���(��)ֵ����λ��ǰ
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////         
INT8 PcdValue(UINT8 dd_mode,UINT8 addr,UINT8 *pValue)
{
    INT8 status;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 
  
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        memcpy(ucComMF522Buf, pValue, 4);
 //       for (i=0; i<16; i++)
 //       {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//��  �ܣ�����Ǯ��
//����˵��: sourceaddr[IN]��Դ��ַ
//      goaladdr[IN]��Ŀ���ַ
//��  ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdBakValue(UINT8 sourceaddr, UINT8 goaladdr)
{
    INT8 status;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}

void RFID_initial(void)
{
	PcdReset();
    PcdAntennaOff(); 
	delay_ms(20);
    PcdAntennaOn();
	delay_ms(20);
	//WriteRawRC(SerialSpeedReg,0x9A);//57600
	//WriteRawRC(SerialSpeedReg,0xAB);//38400
	WriteRawRC(SerialSpeedReg,0xCB);//19200
	delay_ms(20);
}
UINT8 rc_write_data[16] = {0x12,0x34,0x56,0x78,0xED,0xCB,0xA9,0x87,0x12,0x34,0x56,0x78,0x01,0xFE,0x01,0xFE};
//M1����ĳһ��дΪ���¸�ʽ����ÿ�ΪǮ�����ɽ��տۿ�ͳ�ֵ����
//4�ֽڽ����ֽ���ǰ����4�ֽڽ��ȡ����4�ֽڽ�1�ֽڿ��ַ��1�ֽڿ��ַȡ����1�ֽڿ��ַ��1�ֽڿ��ַȡ�� 
UINT8 data2[4]  = {0x12,0,0,0};
UINT8 DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   
UINT8 g_ucTempbuf[20];

UINT8 RFID_SCAN(void)
{
	INT8 status,i;
	UINT16 SXX_ID;	
	
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//Ѱ��
	if(sys.status == ERROR)
    {
		return 0;
    }
    if (status != MI_OK)
    {   
		serail_number = 0;
		return 0;
	}

	status = PcdAnticoll(g_ucTempbuf);//���ص�
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	} 
    
	status = PcdSelect(g_ucTempbuf);//ѡ��
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	}     
         
	status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);//��֤
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	} 
         
    if(1 ==  rc522_write_falg)
	{	
		rc_write_data[0]  = Rfid_Nom&0xFF;
		rc_write_data[1]  = (Rfid_Nom>>8)&0xFF;
		rc_write_data[2]  = 0x00;
		rc_write_data[3]  = 0x00;
		rc_write_data[4]  = ~rc_write_data[0];
		rc_write_data[5]  = ~rc_write_data[1];
		rc_write_data[6]  = ~rc_write_data[2];
		rc_write_data[7]  = ~rc_write_data[3];
		rc_write_data[8]  = rc_write_data[0];
		rc_write_data[9]  = rc_write_data[1];
		rc_write_data[10] = rc_write_data[2];
		rc_write_data[11] = rc_write_data[3];
		rc_write_data[12] = 0x01;
		rc_write_data[13] = 0xFE;
		rc_write_data[14] = 0x01;
		rc_write_data[15] = 0xFE;
	    status = PcdWrite(1, rc_write_data);//д
	    if (status != MI_OK)
		{
		   rc522_write_ret_falg = 1; 
	       return 0; 
		}
		else rc522_write_ret_falg = 0;  
		
	}

	else
	{    
    	status = PcdRead(1, g_ucTempbuf);//��
    	if (status != MI_OK)
		{
	   		serail_number = 0;
	   		return 0;  
		} 
		else
		{
	       serail_number =(UINT16)g_ucTempbuf[0];
		   serail_number =serail_number|((UINT16)g_ucTempbuf[1])<<8;
		 	while(serail_number>999)
			{
				serail_number-=999;
			}
		}
	}
	
    PcdHalt();	
	return 0;
}