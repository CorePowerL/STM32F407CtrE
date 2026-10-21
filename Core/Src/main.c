/* USER CODE BEGIN Header */
/**
  * @brief   6通道PWM发生器HMI - 320x240 横屏, 触摸选通道调占空比, 驱动舵机
  *
  * 通道分配(避开触摸引脚):
  *   CH1: TIM3_CH1  PB4
  *   CH2: TIM3_CH2  PB5
  *   CH3: TIM4_CH1  PB6
  *   CH4: TIM4_CH2  PB7
  *   CH5: TIM5_CH1  PA0
  *   CH6: TIM5_CH2  PA1
  *
  * 触摸XPT2046(软件SPI):
  *   PEN=PC5, CS=PB12, CLK=PB13, MISO=PB14, MOSI=PB15
  *
  * 舵机标准: 50Hz, 0.5~2.5ms脉宽对应0~180度
  */
/* USER CODE END Header */
#include "main.h"
#include "gpio.h"
#include "fsmc.h"
#include "lcd.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

#define SCR_W   320
#define SCR_H   240
#define APP_TASK_STACK    512
#define APP_TASK_PRIO     3

/* PWM时基: 84MHz/(200*8400)=50Hz */
#define TIM_PSC   200
#define TIM_ARR   8400

static uint8_t pwm_duty[6] = {50, 50, 50, 50, 50, 50};
static uint8_t selected_ch = 0;

static int16_t wave_buf[120];
static uint16_t wave_idx = 0;
static uint16_t wave_phase = 0;

uint8_t hour = 10, min = 30, sec = 0;

typedef struct { uint16_t x1,y1,x2,y2; uint8_t id; } Button_t;
static Button_t btn_ch[6] = {
  {240,40,310,62,0}, {240,68,310,90,1}, {240,96,310,118,2},
  {240,124,310,146,3}, {240,152,310,174,4}, {240,180,310,202,5},
};
static Button_t btn_minus = {10,210,55,232,10};
static Button_t btn_plus  = {65,210,110,232,11};

static void PWM_GPIO_Init(void);
static void TIM_ConfigPWM(TIM_TypeDef *t, uint32_t c1, uint32_t c2);
static void PWM_InitAll(void);
static void PWM_SetDuty(uint8_t ch, uint8_t duty);
static void UART1_Init(void);
static void tp_init(void);
static uint8_t tp_get_point(uint16_t *x, uint16_t *y);
static void TP_Scan(void);
static void DrawStaticUI(void);
static void DrawChannelInfo(void);
static void DrawWaveform(void);
static void App_Task(void *pvParameters);
void SystemClock_Config(void);

/* ====================== PWM ====================== */
static void PWM_GPIO_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;

  /* PA1/PA2/PA3 AF2=TIM5_CH2/CH3/CH4 */
  GPIOA->MODER &= ~((3<<2)|(3<<4)|(3<<6));
  GPIOA->MODER |=  ((2<<2)|(2<<4)|(2<<6));
  GPIOA->OSPEEDR |= ((3<<2)|(3<<4)|(3<<6));
  GPIOA->AFR[0] &= ~((0xF<<4)|(0xF<<8)|(0xF<<12));
  GPIOA->AFR[0] |=  ((2<<4)|(2<<8)|(2<<12));

  /* PA6/PA7 AF2=TIM3_CH1/CH2 */
  GPIOA->MODER &= ~((3<<12)|(3<<14));
  GPIOA->MODER |=  ((2<<12)|(2<<14));
  GPIOA->OSPEEDR |= ((3<<12)|(3<<14));
  GPIOA->AFR[0] &= ~((0xF<<24)|(0xF<<28));
  GPIOA->AFR[0] |=  ((2<<24)|(2<<28));

  /* PB0 AF2=TIM3_CH3 */
  GPIOB->MODER &= ~(3<<0);
  GPIOB->MODER |=  (2<<0);
  GPIOB->OSPEEDR |= (3<<0);
  GPIOB->AFR[0] &= ~(0xF<<0);
  GPIOB->AFR[0] |=  (2<<0);
}

static void TIM_ConfigPWM3(void)
{
  TIM3->PSC = TIM_PSC - 1;
  TIM3->ARR = TIM_ARR - 1;
  TIM3->CR1 = TIM_CR1_ARPE;
  /* CH1=PA6, CH2=PA7, CH3=PB0, PWM mode1 */
  TIM3->CCMR1 = (6<<4)|(1<<3) | (6<<12)|(1<<11);
  TIM3->CCMR2 = (6<<4)|(1<<3);
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  uint32_t half = TIM_ARR/2;
  TIM3->CCR1 = half; TIM3->CCR2 = half; TIM3->CCR3 = half;
  TIM3->CR1 |= TIM_CR1_CEN;
}

static void TIM_ConfigPWM5(void)
{
  TIM5->PSC = TIM_PSC - 1;
  TIM5->ARR = TIM_ARR - 1;
  TIM5->CR1 = TIM_CR1_ARPE;
  /* CH2=PA1, CH3=PA2, CH4=PA3, PWM mode1 */
  TIM5->CCMR1 = (6<<12)|(1<<11);
  TIM5->CCMR2 = (6<<4)|(1<<3) | (6<<12)|(1<<11);
  TIM5->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  uint32_t half = TIM_ARR/2;
  TIM5->CCR2 = half; TIM5->CCR3 = half; TIM5->CCR4 = half;
  TIM5->CR1 |= TIM_CR1_CEN;
}

static void PWM_InitAll(void)
{
  PWM_GPIO_Init();
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN;
  TIM_ConfigPWM3();
  TIM_ConfigPWM5();
}

static void PWM_SetDuty(uint8_t ch, uint8_t duty)
{
  /* duty 0~100% -> 舵机脉宽 1.0ms~2.0ms (兼容MG996R/SG90)
     ARR=8400, 计数时钟~418kHz, 每tick~2.39us
     1.0ms=418tick, 2.0ms=836tick */
  if (duty > 100) duty = 100;
  pwm_duty[ch] = duty;
  uint32_t ccr = 418 + (uint32_t)duty * 418 / 100;
  switch (ch) {
    case 0: TIM5->CCR2=ccr; break;  /* PA1 */
    case 1: TIM5->CCR3=ccr; break;  /* PA2 */
    case 2: TIM5->CCR4=ccr; break;  /* PA3 */
    case 3: TIM3->CCR1=ccr; break;  /* PA6 */
    case 4: TIM3->CCR2=ccr; break;  /* PA7 */
    case 5: TIM3->CCR3=ccr; break;  /* PB0 */
  }
}

/* ====================== USART1 调试串口 (PA9=TX, PA10=RX, 115200) ====================== */
static void UART1_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /* PA9/PA10 AF7=USART1 */
  GPIOA->MODER &= ~((3<<18)|(3<<20));
  GPIOA->MODER |=  ((2<<18)|(2<<20));
  GPIOA->OSPEEDR |= ((3<<18)|(3<<20));
  GPIOA->AFR[1] &= ~((0xF<<(9-8)*4)|(0xF<<(10-8)*4));
  GPIOA->AFR[1] |=  ((7<<(9-8)*4)|(7<<(10-8)*4));

  /* 84MHz, 115200: BRR = 84M/115200 = 729.17 */
  USART1->BRR = 729;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* printf 重定向到 USART1 */
int fputc(int ch, FILE *f)
{
  (void)f;
  while (!(USART1->SR & USART_SR_TXE));
  USART1->DR = (uint8_t)ch;
  return ch;
}

/* ====================== 触摸 XPT2046 软件SPI ====================== */
/* PEN=PC5, CS=PB12, CLK=PB13, MISO=PB14, MOSI=PB15 */
#define TP_PEN()      (GPIOC->IDR & GPIO_PIN_5)
#define TP_CS_H()     (GPIOB->BSRR = GPIO_PIN_12)
#define TP_CS_L()     (GPIOB->BSRR = GPIO_PIN_12<<16)
#define TP_CLK_H()    (GPIOB->BSRR = GPIO_PIN_13)
#define TP_CLK_L()    (GPIOB->BSRR = GPIO_PIN_13<<16)
#define TP_MOSI_H()   (GPIOB->BSRR = GPIO_PIN_15)
#define TP_MOSI_L()   (GPIOB->BSRR = GPIO_PIN_15<<16)
#define TP_MISO()     (GPIOB->IDR & GPIO_PIN_14)

static void tp_udelay(void) { for (volatile int i=0;i<8;i++); }

static void tp_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  /* PC5=PEN 上拉输入 */
  GPIOC->MODER &= ~(3<<10);
  GPIOC->PUPDR &= ~(3<<10);
  GPIOC->PUPDR |=  (1<<10);
  /* PB14=MISO 上拉输入 */
  GPIOB->MODER &= ~(3<<28);
  GPIOB->PUPDR &= ~(3<<28);
  GPIOB->PUPDR |=  (1<<28);
  /* PB12=CS PB13=CLK PB15=MOSI 推挽输出 */
  GPIOB->MODER &= ~((3<<24)|(3<<26)|(3<<30));
  GPIOB->MODER |=  ((1<<24)|(1<<26)|(1<<30));
  GPIOB->OSPEEDR |= ((3<<24)|(3<<26)|(3<<30));
  TP_CS_H(); TP_CLK_H();
}

static uint16_t tp_adc(uint8_t cmd)
{
  uint16_t v=0;
  TP_CS_L(); tp_udelay();
  for (int i=0;i<8;i++) {
    if (cmd&0x80) TP_MOSI_H(); else TP_MOSI_L();
    cmd<<=1;
    TP_CLK_L(); tp_udelay(); TP_CLK_H(); tp_udelay();
  }
  tp_udelay();
  TP_CLK_L(); tp_udelay(); TP_CLK_H(); tp_udelay(); TP_CLK_L();
  for (int i=0;i<12;i++) {
    v<<=1;
    TP_CLK_L(); tp_udelay();
    if (TP_MISO()) v++;
    TP_CLK_H(); tp_udelay();
  }
  TP_CS_H();
  return v;
}

static uint8_t tp_get_point(uint16_t *x, uint16_t *y)
{
  uint16_t ax, ay;
  if (TP_PEN()) return 0;
  ax = tp_adc(0x90);
  ay = tp_adc(0xD0);
  /* 五点校准后映射:
     x: ax=3700->0, ax=450->319
     y: ay=3725->0, ay=490->239 */
  if (ax > 3700) ax = 3700;
  if (ay > 3725) ay = 3725;
  *x = (uint32_t)(ax - 450) * 319 / 3250;
  *y = (uint32_t)(3725 - ay) * 239 / 3235;
  printf("raw: ax=%d ay=%d -> x=%d y=%d\n", ax, ay, *x, *y);
  return 1;
}

static void TP_Scan(void)
{
  uint16_t x,y;
  if (!tp_get_point(&x,&y)) return;
  printf("TP: x=%d y=%d\n", x, y);
  for (int i=0;i<6;i++) {
    if (x>=btn_ch[i].x1 && x<=btn_ch[i].x2 && y>=btn_ch[i].y1 && y<=btn_ch[i].y2) {
      selected_ch = i; DrawChannelInfo(); return;
    }
  }
  if (x>=btn_minus.x1 && x<=btn_minus.x2 && y>=btn_minus.y1 && y<=btn_minus.y2) {
    if (pwm_duty[selected_ch]>=5) { PWM_SetDuty(selected_ch, pwm_duty[selected_ch]-5); DrawChannelInfo(); }
    return;
  }
  if (x>=btn_plus.x1 && x<=btn_plus.x2 && y>=btn_plus.y1 && y<=btn_plus.y2) {
    if (pwm_duty[selected_ch]<=95) { PWM_SetDuty(selected_ch, pwm_duty[selected_ch]+5); DrawChannelInfo(); }
    return;
  }
}

/* ====================== UI ====================== */
static void DrawStaticUI(void)
{
  char s[16];
  lcd_fill(0,0,SCR_W-1,22,0x2104);
  lcd_show_string(5,3,120,16,16,"PWM Generator",CYAN);
  lcd_show_string(240,3,75,16,16,"50Hz",GREEN);

  lcd_draw_rectangle(2,28,100,100,CYAN);
  lcd_show_string(6,30,90,10,12,"CHANNEL",GRAY);

  lcd_draw_rectangle(2,108,100,200,GREEN);
  lcd_show_string(6,110,90,10,12,"DUTY %",GRAY);

  lcd_draw_rectangle(105,28,230,200,CYAN);
  lcd_show_string(108,30,80,10,12,"WAVEFORM",GRAY);

  for (int i=0;i<6;i++) {
    lcd_draw_rectangle(btn_ch[i].x1,btn_ch[i].y1,btn_ch[i].x2,btn_ch[i].y2,GRAY);
    sprintf(s,"CH%d",i+1);
    lcd_show_string(btn_ch[i].x1+15,btn_ch[i].y1+4,50,12,16,s,CYAN);
  }

  lcd_fill(0,205,SCR_W-1,SCR_H-1,0x2104);
  lcd_draw_rectangle(btn_minus.x1,btn_minus.y1,btn_minus.x2,btn_minus.y2,RED);
  lcd_show_string(btn_minus.x1+15,btn_minus.y1+4,30,12,16,"-",RED);
  lcd_draw_rectangle(btn_plus.x1,btn_plus.y1,btn_plus.x2,btn_plus.y2,GREEN);
  lcd_show_string(btn_plus.x1+18,btn_plus.y1+4,30,12,16,"+",GREEN);
}

static void DrawChannelInfo(void)
{
  char s[16];
  lcd_fill(20,45,95,75,BLACK);
  sprintf(s,"CH%d",selected_ch+1);
  lcd_show_string(20,45,70,24,24,s,CYAN);
  lcd_fill(8,125,95,175,BLACK);
  uint8_t angle = pwm_duty[selected_ch]*18/10;
  sprintf(s,"%d deg",angle);
  lcd_show_string(12,130,80,24,24,s,GREEN);
  lcd_show_string(8,178,90,10,12,"pulse:",GRAY);
  sprintf(s,"%.1fms",1.0f+(float)pwm_duty[selected_ch]*1.0f/100.0f);
  lcd_show_string(50,178,50,10,12,s,YELLOW);
}

static void DrawWaveform(void)
{
  uint8_t duty = pwm_duty[selected_ch];
  uint8_t hi = duty*120/100;
  uint16_t old = wave_idx;

  for (uint16_t i=0;i<119;i++) {
    uint16_t i1=(old+i)%120, i2=(old+i+1)%120;
    lcd_draw_line(106+i,80+wave_buf[i1],106+i+1,80+wave_buf[i2],BLACK);
  }
  wave_phase++;
  if (wave_phase>=120) wave_phase=0;
  wave_buf[wave_idx] = (wave_phase<hi) ? -30 : 40;
  wave_idx++;
  if (wave_idx>=120) wave_idx=0;
  for (uint16_t i=0;i<119;i++) {
    uint16_t i1=(wave_idx+i)%120, i2=(wave_idx+i+1)%120;
    lcd_draw_line(106+i,80+wave_buf[i1],106+i+1,80+wave_buf[i2],GREEN);
  }
}

/* ====================== 任务 ====================== */
static void App_Task(void *pv)
{
  char buf[16];
  TickType_t lt=0, lw=0, now;
  (void)pv;

  for (int i=0;i<120;i++) wave_buf[i]=40;
  for (uint8_t ch=0;ch<6;ch++) PWM_SetDuty(ch, pwm_duty[ch]);
  DrawChannelInfo();
  lt = xTaskGetTickCount();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10));
    now = xTaskGetTickCount();

    TP_Scan();

    if ((now-lt)>=1000) {
      lt=now; sec++;
      if (sec>=60){sec=0;min++;}
      if (min>=60){min=0;hour++;}
      if (hour>=24) hour=0;
      lcd_fill(240,3,317,20,0x2104);
      sprintf(buf,"%02d:%02d:%02d",hour,min,sec);
      lcd_show_string(240,3,75,16,16,buf,GREEN);
    }
    if ((now-lw)>=30) { lw=now; DrawWaveform(); }
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_FSMC_Init();
  PWM_InitAll();
  tp_init();
  UART1_Init();
  printf("BOOT OK\n");

  DWT_Init();
  lcd_init();
  lcd_dma_init();
  lcd_clear(BLACK);
  DrawStaticUI();

  xTaskCreate(App_Task,"App",APP_TASK_STACK,NULL,APP_TASK_PRIO,NULL);
  vTaskStartScheduler();
  while(1);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o={0};
  RCC_ClkInitTypeDef c={0};
  __HAL_RCC_PWR_CLK_ENABLE();
  o.OscillatorType=RCC_OSCILLATORTYPE_HSI;
  o.HSIState=RCC_HSI_ON;
  o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState=RCC_PLL_ON;
  o.PLL.PLLSource=RCC_PLLSOURCE_HSI;
  o.PLL.PLLM=8; o.PLL.PLLN=168; o.PLL.PLLP=RCC_PLLP_DIV2; o.PLL.PLLQ=4;
  HAL_RCC_OscConfig(&o);
  c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
  c.AHBCLKDivider=RCC_SYSCLK_DIV1;
  c.APB1CLKDivider=RCC_HCLK_DIV4;
  c.APB2CLKDivider=RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&c,FLASH_LATENCY_5);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n){ (void)t;(void)n; while(1); }
void Error_Handler(void){ while(1); }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l){ (void)f;(void)l; }
#endif
