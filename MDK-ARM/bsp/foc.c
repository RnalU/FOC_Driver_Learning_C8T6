#include "foc.h"

//变量定义部分
Trig_Components Vector_Components;
u8 bSector;
u8 PWM4Direction=PWM2_MODE;
s16 cnt = S16_MIN;//开环调试变量

//FOC相关
Trig_Components Vector_Components;
Curr_Components Stat_Curr_a_b;            
Curr_Components Stat_Curr_alfa_beta;       
Curr_Components Stat_Curr_q_d;             
Curr_Components Stat_Curr_q_d_ref_ref;   //电流环的给定值，用于电流环Id,Iq和前馈电流控制的给定值
Volt_Components Stat_Volt_q_d;             
Volt_Components Stat_Volt_alfa_beta; 
PID_Struct_t PID_Torque_InitStructure;
PID_Struct_t PID_Flux_InitStructure;
PID_Struct_t PID_Speed_InitStructure;

void motor_init(void)
{
	//PWM初始化
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
  //通道4触发ADC采样
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,50);//初始占空比应该多少？
//	//开启ADC注入转换
//	HAL_ADCEx_InjectedStart_IT(&hadc1);
//	//使能ABZ编码器
//	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
//  HAL_TIM_Base_Start_IT(&htim2);
//	//初始化PID控制器
//	PID_Init(&PID_Torque_InitStructure,&PID_Flux_InitStructure,&PID_Speed_InitStructure);
  
//	State = START;
}

void FOC_Model(void)	  //电流环处理函数
{
//	Stat_Curr_a_b = SVPWM_3ShuntGetPhaseCurrentValues();			//读取2相的电流值
//	Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);	//Ia,Ib通过Clark变换得到Ialpha和Ibeta  
//	Stat_Curr_q_d = Park( Stat_Curr_alfa_beta,ENC_Get_Electrical_Angle());  //输入电角度、Ialpha和Ibeta，经过Park变换得到Iq、Id							
//	Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference,Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
//	Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference,Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);  	
//	RevPark_Circle_Limitation();  	//归一化
	
	
	//开环调试
	Stat_Volt_q_d.qV_Component1 = 0;
	Stat_Volt_q_d.qV_Component2 = 3000;
	cnt+=500;
	if(cnt>S16_MAX)
		cnt=S16_MIN;
	Vector_Components = Trig_Functions(cnt);
	Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);    //反Park变换
	SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);  //svpwm实现函数，实际的电流输出控制	
}




//SVPWM
void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
   s32 wX, wY, wZ, wUAlpha, wUBeta;
   u16  hTimePhA=0, hTimePhB=0, hTimePhC=0, hTimePhD=0;
   u16  hDeltaDuty;
    
   wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
   wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

   wX = wUBeta;
   wY = (wUBeta + wUAlpha)/2;
   wZ = (wUBeta - wUAlpha)/2;
   
  // Sector calculation from wX, wY, wZ
   if (wY<0)
   {
      if (wZ<0)
      {
        bSector = SECTOR_5;
      }
      else // wZ >= 0
        if (wX<=0)
        {
          bSector = SECTOR_4;
        }
        else // wX > 0
        {
          bSector = SECTOR_3;
        }
   }
   else // wY > 0
   {
     if (wZ>=0)
     {
       bSector = SECTOR_2;
     }
     else // wZ < 0
       if (wX<=0)
       {  
         bSector = SECTOR_6;
       }
       else // wX > 0
       {
         bSector = SECTOR_1;
       }
    }
   
   /* Duty cycles computation */
  PWM4Direction=PWM2_MODE;
    
  switch(bSector)
  {  
    case SECTOR_1:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhB - wX/131072;
                
                // ADC Syncronization setting value             
                if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhA - hTimePhB);
                  
				  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
                  {
                      hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
                  }
                  else
                  {
                      hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A
                     
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }
                            
                break;
    case SECTOR_2:
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhA - wY/131072;
                
                // ADC Syncronization setting value
                if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhB - hTimePhA);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
                  {
                    hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
                  }
                  else
                  {
                    hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }
                
                break;

    case SECTOR_3:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
				        hTimePhC = hTimePhA - wY/131072;
                hTimePhB = hTimePhC + wX/131072;
		
                // ADC Syncronization setting value
                if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhB - hTimePhC);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
                  {
                    hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
                  }
                  else
                  {
                    hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }
                break;
    
    case SECTOR_4:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
                hTimePhB = hTimePhA + wZ/131072;
                hTimePhC = hTimePhB - wX/131072;
                
                // ADC Syncronization setting value
                if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhC - hTimePhB);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2)
                  {
                    hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
                  }
                  else
                  {
                    hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }
                break;  
    
    case SECTOR_5:
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
				hTimePhB = hTimePhA + wZ/131072;
				hTimePhC = hTimePhA - wY/131072;
                
                // ADC Syncronization setting value
                if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhC - hTimePhA);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2) 
                  {
                    hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
                  }
                  else
                  {
                    hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }

		break;
                
    case SECTOR_6:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
				hTimePhC = hTimePhA - wY/131072;
				hTimePhB = hTimePhC + wX/131072;
                
                // ADC Syncronization setting value
                if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (u16)(hTimePhA - hTimePhC);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
                  {
                    hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
                  }
                  else
                  {
                    hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=PWM1_MODE;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
                }

                break;
    default:
		break;
   }
  
  if (PWM4Direction == PWM2_MODE)
  {
    //Set Polarity of CC4 High
    TIM1->CCER &= 0xDFFF;    
  }
  else
  {
    //Set Polarity of CC4 Low
    TIM1->CCER |= 0x2000;
  }
  
  /* Load compare registers values */ 
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,hTimePhA);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,hTimePhB);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,hTimePhC);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,hTimePhD);
}

//数学函数
const s16 hSin_Cos_Table[256] = {\
0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE};



Curr_Components Clarke(Curr_Components Curr_Input)         
{
  Curr_Components Curr_Output; 
  s32 qIa_divSQRT3_tmp;
  s32 qIb_divSQRT3_tmp;    //定义32位有符号数，用来暂存Q30格式  
  s16 qIa_divSQRT3;
  s16 qIb_divSQRT3 ;
 
  Curr_Output.qI_Component1 = Curr_Input.qI_Component1;     //Ialpha = Ia
 
  qIa_divSQRT3_tmp = divSQRT_3 * Curr_Input.qI_Component1;  //计算Ia/√3
  qIa_divSQRT3_tmp /=32768;                                 //两个Q15数相乘，会变成Q30，因此要右移15位，变回Q15	    
  qIb_divSQRT3_tmp = divSQRT_3 * Curr_Input.qI_Component2;  //计算Ib/√3
  qIb_divSQRT3_tmp /=32768;
  
  qIa_divSQRT3=((s16)(qIa_divSQRT3_tmp));	                //s32赋值给s16	 		
  qIb_divSQRT3=((s16)(qIb_divSQRT3_tmp));				
     
  Curr_Output.qI_Component2=(-(qIa_divSQRT3)-(qIb_divSQRT3)-(qIb_divSQRT3));  //Ibeta = -(2*Ib+Ia)/sqrt(3) 
  return(Curr_Output); 
}
 
	
/*******************************************************************************
* Function Name  : Trig_Functions 
* Description    : 本函数返回输入角度的cos和sin函数值
* Input          : angle in s16 format
* Output         : Cosine and Sine in s16 format
*******************************************************************************/
Trig_Components Trig_Functions(s16 hAngle)  //hAngle=0,转子电角度=0度。hAngle=S16_MAX,转子电角度=180度。hAngle=S16_MIN,转子电角度=-180度
{
  u16 hindex;
  Trig_Components Local_Components;
  
  /* 10 bit index computation  */  
  hindex = (u16)(hAngle + 32768);  
  hindex /= 64;      
  
  switch (hindex & SIN_MASK) 
  {
  case U0_90:
    Local_Components.hSin = hSin_Cos_Table[(u8)(hindex)];
    Local_Components.hCos = hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
    break;
  
  case U90_180:  
     Local_Components.hSin = hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
     Local_Components.hCos = -hSin_Cos_Table[(u8)(hindex)];
    break;
  
  case U180_270:
     Local_Components.hSin = -hSin_Cos_Table[(u8)(hindex)];
     Local_Components.hCos = -hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
    break;
  
  case U270_360:
     Local_Components.hSin =  -hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
     Local_Components.hCos =  hSin_Cos_Table[(u8)(hindex)]; 
    break;
  default:
    break;
  }
  return (Local_Components);
}
 

/**********************************************************************************************************
Park变换，输入电角度、Ialpha和Ibeta，经过Park变换得到Iq、Id
**********************************************************************************************************/ 
Curr_Components Park(Curr_Components Curr_Input, s16 Theta)       
{ 
  Curr_Components Curr_Output;
  s32 qId_tmp_1, qId_tmp_2;
  s32 qIq_tmp_1, qIq_tmp_2;     
  s16 qId_1, qId_2;  
  s16 qIq_1, qIq_2;  
  
  Vector_Components = Trig_Functions(Theta);                      //计算电角度的cos和sin
  
  qIq_tmp_1 = Curr_Input.qI_Component1 * Vector_Components.hCos;  //计算Ialpha*cosθ	
  qIq_tmp_1 /= 32768;
  qIq_tmp_2 = Curr_Input.qI_Component2 *Vector_Components.hSin;   //计算Ibeta*sinθ
  qIq_tmp_2 /= 32768;
 
  qIq_1 = ((s16)(qIq_tmp_1));
  qIq_2 = ((s16)(qIq_tmp_2));
  Curr_Output.qI_Component1 = ((qIq_1)-(qIq_2));	//Iq=Ialpha*cosθ- Ibeta*sinθ
  
  qId_tmp_1 = Curr_Input.qI_Component1 * Vector_Components.hSin;  //计算Ialpha*sinθ
  qId_tmp_1 /= 32768;
  qId_tmp_2 = Curr_Input.qI_Component2 * Vector_Components.hCos;  //计算Ibeta*cosθ
  qId_tmp_2 /= 32768;
  
  qId_1 = (s16)(qId_tmp_1);		 
  qId_2 = (s16)(qId_tmp_2);					
  Curr_Output.qI_Component2 = ((qId_1)+(qId_2));   //Id=Ialpha*sinθ+ Ibeta*cosθ
  
  return (Curr_Output);
}
 
/**********************************************************************************************************
反park变换，输入Uq、Ud得到Ualpha、Ubeta
**********************************************************************************************************/ 
Volt_Components Rev_Park(Volt_Components Volt_Input)
{ 	
  s32 qValpha_tmp1,qValpha_tmp2,qVbeta_tmp1,qVbeta_tmp2;
  s16 qValpha_1,qValpha_2,qVbeta_1,qVbeta_2;
  Volt_Components Volt_Output;
   
  qValpha_tmp1 = Volt_Input.qV_Component1 * Vector_Components.hCos;  //Uq*cosθ
  qValpha_tmp1 /= 32768; 
  qValpha_tmp2 = Volt_Input.qV_Component2 * Vector_Components.hSin;  //Ud*sinθ
  qValpha_tmp2 /= 32768;
		
  qValpha_1 = (s16)(qValpha_tmp1);		
  qValpha_2 = (s16)(qValpha_tmp2);			
  Volt_Output.qV_Component1 = ((qValpha_1)+(qValpha_2));             //Ualpha=Uq*cosθ+ Ud*sinθ
  
  qVbeta_tmp1 = Volt_Input.qV_Component1 * Vector_Components.hSin;   //Uq*sinθ
  qVbeta_tmp1 /= 32768;  
  qVbeta_tmp2 = Volt_Input.qV_Component2 * Vector_Components.hCos;   //Ud*cosθ
  qVbeta_tmp2 /= 32768;
 
  qVbeta_1 = (s16)(qVbeta_tmp1);				
  qVbeta_2 = (s16)(qVbeta_tmp2);  				
  Volt_Output.qV_Component2 = -(qVbeta_1)+(qVbeta_2);                //Ubeta=Ud*cosθ- Uq*sinθ
 
  return(Volt_Output);
}

