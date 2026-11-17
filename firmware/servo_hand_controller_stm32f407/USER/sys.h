#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 

//位带公式
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//GPIO输入输出寄存器地址+偏移地址
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO?ú2ù×÷,????μ￥ò?μ?IO?ú!
//è·±￡nμ??μD?óú16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //ê?3? 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //ê?è? 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //ê?3? 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //ê?è? 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //ê?3? 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //ê?è? 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //ê?3? 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //ê?è? 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //ê?3? 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //ê?è?

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //ê?3?  
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //ê?è?

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //ê?3? 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //ê?è?

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //ê?3? 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //ê?è?

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //ê?3? 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //ê?è?


#endif

