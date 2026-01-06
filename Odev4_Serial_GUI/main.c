#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

// ---------------------------------------------------------
//  LCD PIN EŞLEME
// ---------------------------------------------------------
#define LCD_PORT_RS_CLK   SYSCTL_PERIPH_GPIOA
#define LCD_PORT_RS_BASE  GPIO_PORTA_BASE
#define LCD_PIN_RS        GPIO_PIN_5     // RS -> PA5

#define LCD_PORT_E_CLK    SYSCTL_PERIPH_GPIOA
#define LCD_PORT_E_BASE   GPIO_PORTA_BASE
#define LCD_PIN_E         GPIO_PIN_6     // E  -> PA6

#define LCD_PORT_D_CLK    SYSCTL_PERIPH_GPIOE
#define LCD_PORT_D_BASE   GPIO_PORTE_BASE
#define LCD_PIN_D4        GPIO_PIN_1     // D4 -> PE1
#define LCD_PIN_D5        GPIO_PIN_2     // D5 -> PE2
#define LCD_PIN_D6        GPIO_PIN_3     // D6 -> PE3
#define LCD_PIN_D7        GPIO_PIN_4     // D7 -> PE4

// ---------------------------------------------------------
//  GECIKME FONKSIYONLARI
// ---------------------------------------------------------
static inline void delay_us(uint32_t us) {
    SysCtlDelay((SysCtlClockGet() / 3000000) * us); // ~1us @80MHz
}
static inline void delay_ms(uint32_t ms){
    while(ms--) delay_us(1000);
}

// ---------------------------------------------------------
//  LCD TEMEL FONKSIYONLARI
// ---------------------------------------------------------
static void pulseE(void){
    GPIOPinWrite(LCD_PORT_E_BASE, LCD_PIN_E, LCD_PIN_E);
    delay_us(2);
    GPIOPinWrite(LCD_PORT_E_BASE, LCD_PIN_E, 0);
    delay_us(50);
}

static void write4(uint8_t nibble){
    GPIOPinWrite(LCD_PORT_D_BASE,
                 LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7, 0);

    {
        uint8_t out = 0;
        if(nibble & 0x1) out |= LCD_PIN_D4;
        if(nibble & 0x2) out |= LCD_PIN_D5;
        if(nibble & 0x4) out |= LCD_PIN_D6;
        if(nibble & 0x8) out |= LCD_PIN_D7;

        GPIOPinWrite(LCD_PORT_D_BASE,
                     LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7, out);
    }
    pulseE();
}

static void send8(uint8_t val, bool rs){
    GPIOPinWrite(LCD_PORT_RS_BASE, LCD_PIN_RS, rs ? LCD_PIN_RS : 0);
    write4((val >> 4) & 0x0F);
    write4(val & 0x0F);
}

static void cmd(uint8_t c){
    send8(c, false);
    if(c==0x01 || c==0x02) delay_ms(2);
    else delay_us(50);
}

static void putc_lcd(char ch){ send8((uint8_t)ch, true); }

static void lcd_puts(const char *s){
    while(*s) putc_lcd(*s++);
}

static void lcd_goto(uint8_t col, uint8_t row){
    const uint8_t offsets[] = {0x00, 0x40};
    cmd(0x80 | (col + offsets[row]));
}

static void lcd_clear_line(uint8_t row){
    int i;
    lcd_goto(0,row);
    for(i=0;i<16;i++) putc_lcd(' ');
}

// LCD başlatma
static void lcd_init(void){
    // Clock ayarı
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL |
                   SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(LCD_PORT_RS_CLK);
    SysCtlPeripheralEnable(LCD_PORT_E_CLK);
    SysCtlPeripheralEnable(LCD_PORT_D_CLK);
    while(!SysCtlPeripheralReady(LCD_PORT_RS_CLK) ||
          !SysCtlPeripheralReady(LCD_PORT_E_CLK) ||
          !SysCtlPeripheralReady(LCD_PORT_D_CLK));

    GPIOPinTypeGPIOOutput(LCD_PORT_RS_BASE, LCD_PIN_RS);
    GPIOPinTypeGPIOOutput(LCD_PORT_E_BASE,  LCD_PIN_E);
    GPIOPinTypeGPIOOutput(LCD_PORT_D_BASE,
                          LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7);

    GPIOPinWrite(LCD_PORT_RS_BASE, LCD_PIN_RS, 0);
    GPIOPinWrite(LCD_PORT_E_BASE,  LCD_PIN_E,  0);

    delay_ms(30);
    write4(0x03); delay_ms(5);
    write4(0x03); delay_ms(5);
    write4(0x03); delay_ms(5);
    write4(0x02); delay_ms(5);

    cmd(0x28); // 4-bit, 2 satır
    cmd(0x0C); // display ON, cursor OFF
    cmd(0x06); // entry mode
    cmd(0x01); // clear
    delay_ms(2);
}

// ---------------------------------------------------------
//  SAYI YAZDIRMA (GENEL)
// ---------------------------------------------------------
static void lcd_put_int(int32_t v){
    char buf[12];
    int i = 0;
    bool neg = false;

    if(v == 0){
        putc_lcd('0');
        return;
    }
    if(v < 0){
        neg = true;
        v = -v;
    }
    while(v > 0 && i < (int)(sizeof(buf)-1)){
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    if(neg) putc_lcd('-');
    while(i--){
        putc_lcd(buf[i]);
    }
}

// 2 haneli yazdır (00-99)
static void lcd_put_2d(uint8_t v){
    putc_lcd('0' + (v / 10));
    putc_lcd('0' + (v % 10));
}

// ---------------------------------------------------------
//  ADC: DAHILI SICAKLIK SENSÖRÜ
// ---------------------------------------------------------
static void adc_init_temp(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

static uint32_t adc_read_temp_raw(void){
    uint32_t val;

    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false));
    ADCSequenceDataGet(ADC0_BASE, 3, &val);

    return val; // 0..4095
}

// Temp(C) ≈ 147.5 - (75 * 3.3 * ADC / 4096)
// 10x ile → 1475 ve 2475
static int32_t temp_raw_to_c10(uint32_t adcVal){
    int32_t t10 = 1475 - ( (2475 * (int32_t)adcVal) / 4096 );
    return t10;
}

// ALT SATIR: "S: xx.xC"
static void temp_print(int32_t t10){
    int32_t t_int  = t10 / 10;
    int32_t t_frac = t10 % 10;
    if(t_frac < 0) t_frac = -t_frac;

    lcd_goto(0,1);
    lcd_puts("S:");
    lcd_goto(3,1);
    lcd_put_int(t_int);
    putc_lcd('.');
    putc_lcd('0' + (int)t_frac);
    putc_lcd('C');
    lcd_puts("   ");
}

// ---------------------------------------------------------
//  PF4 BUTON
// ---------------------------------------------------------
static void pf4_init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

// PF4 pull-up: basılıyken 0 gelir
static uint8_t pf4_pressed(void){
    return (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0) ? 1 : 0;
}

// ---------------------------------------------------------
//  UART0: PC ILE HABERLEŞME
// ---------------------------------------------------------
static void uart0_init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    {
        uint32_t clk = SysCtlClockGet();
        UARTConfigSetExpClk(UART0_BASE, clk, 115200,
                            UART_CONFIG_WLEN_8 |
                            UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);
    }
}

static void uart0_putc(char c){
    UARTCharPut(UART0_BASE, c);
}

static void uart0_puts(const char *s){
    while(*s) uart0_putc(*s++);
}

static void uart0_put_int(int32_t v){
    char buf[12];
    int i = 0;
    bool neg = false;

    if(v == 0){
        uart0_putc('0');
        return;
    }
    if(v < 0){
        neg = true;
        v = -v;
    }
    while(v > 0 && i < (int)(sizeof(buf)-1)){
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    if(neg) uart0_putc('-');
    while(i--){
        uart0_putc(buf[i]);
    }
}

static void uart0_put_2d(uint8_t v){
    uart0_putc('0' + (v / 10));
    uart0_putc('0' + (v % 10));
}

// Her saniye PC’ye: TIME:HH:MM:SS TEMP:xx.x BTN:0/1
static void uart0_send_status(uint8_t hh, uint8_t mm, uint8_t ss, int32_t t10, uint8_t btn){
    int32_t t_int  = t10 / 10;
    int32_t t_frac = t10 % 10;
    if(t_frac < 0) t_frac = -t_frac;

    uart0_puts("TIME:");
    uart0_put_2d(hh); uart0_putc(':');
    uart0_put_2d(mm); uart0_putc(':');
    uart0_put_2d(ss);

    uart0_puts(" TEMP:");
    uart0_put_int(t_int);
    uart0_putc('.');
    uart0_putc('0' + (int)t_frac);

    uart0_puts(" BTN:");
    uart0_putc(btn ? '1' : '0');
    uart0_putc('\n');
}

// ---------------------------------------------------------
//  SAAT + 3 KARAKTER LCD ÜST SATIR
// ---------------------------------------------------------
static uint8_t g_hh = 0, g_mm = 0, g_ss = 0;
static char g_txt3[4] = {'-','-','-', '\0'};

static void clock_tick_1s(void){
    g_ss++;
    if(g_ss >= 60){ g_ss = 0; g_mm++; }
    if(g_mm >= 60){ g_mm = 0; g_hh++; }
    if(g_hh >= 24){ g_hh = 0; }
}

static void lcd_top_print(void){
    // Format: "HH:MM:SS ABC"
    lcd_goto(0,0);
    lcd_put_2d(g_hh); putc_lcd(':');
    lcd_put_2d(g_mm); putc_lcd(':');
    lcd_put_2d(g_ss); putc_lcd(' ');

    putc_lcd(g_txt3[0]);
    putc_lcd(g_txt3[1]);
    putc_lcd(g_txt3[2]);

    // kalan yeri temizle
    lcd_puts("    ");
}

// ---------------------------------------------------------
//  PC RX KOMUTLARI (DÜZENLENDİ)
//   C#'dan gelen formatlar:
//   SETTIME=HH:MM:SS
//   SETTXT=ABC
// ---------------------------------------------------------
#define RX_BUF_SIZE 64
static char rxBuf[RX_BUF_SIZE];
static uint32_t rxIndex = 0;

static void apply_command_line(const char *line){

    // C# tarafı "SETTIME=" gönderiyor (8 karakter)
    if(strncmp(line, "SETTIME=", 8) == 0){
        uint8_t hh = 0, mm = 0, ss = 0;

        // "HH:MM:SS" parse (SETTIME= sonrası 8. karakterden başlar)
        if(line[8]  >= '0' && line[8]  <= '2' &&
           line[9]  >= '0' && line[9]  <= '9' &&
           line[10] == ':' &&
           line[11] >= '0' && line[11] <= '5' &&
           line[12] >= '0' && line[12] <= '9' &&
           line[13] == ':' &&
           line[14] >= '0' && line[14] <= '5' &&
           line[15] >= '0' && line[15] <= '9')
        {
            hh = (uint8_t)((line[8]-'0')*10 + (line[9]-'0'));
            mm = (uint8_t)((line[11]-'0')*10 + (line[12]-'0'));
            ss = (uint8_t)((line[14]-'0')*10 + (line[15]-'0'));

            if(hh < 24){
                g_hh = hh; g_mm = mm; g_ss = ss;
            }
        }
        return;
    }

    // C# tarafı "SETTXT=" gönderiyor (7 karakter)
    if(strncmp(line, "SETTXT=", 7) == 0){
        // "SETTXT=" sonrası ilk 3 karakteri al (7, 8, 9. indeksler)
        if(line[7] != '\0') g_txt3[0] = line[7];
        if(line[8] != '\0') g_txt3[1] = line[8];
        if(line[9] != '\0') g_txt3[2] = line[9];
        return;
    }
}

static void uart0_process_rx(void){
    while(UARTCharsAvail(UART0_BASE)){
        char c = UARTCharGetNonBlocking(UART0_BASE);

        if(c == '\r' || c == '\n'){
            if(rxIndex > 0){
                rxBuf[rxIndex] = '\0';
                apply_command_line(rxBuf);
                rxIndex = 0;
            }
        } else {
            if(rxIndex < RX_BUF_SIZE-1){
                rxBuf[rxIndex++] = c;
            }
        }
    }
}

// ---------------------------------------------------------
//  MAIN
// ---------------------------------------------------------
int main(void){
    int i;

    lcd_init();
    adc_init_temp();
    uart0_init();
    pf4_init();

    lcd_clear_line(0);
    lcd_clear_line(1);

    // İlk yazdırma
    lcd_top_print();
    temp_print(temp_raw_to_c10(adc_read_temp_raw()));

    while(1){
        uint32_t raw = adc_read_temp_raw();
        int32_t t10  = temp_raw_to_c10(raw);
        uint8_t btn  = pf4_pressed();

        // LCD güncelle
        lcd_top_print();
        temp_print(t10);

        // PC’ye her saniye paket gönder
        uart0_send_status(g_hh, g_mm, g_ss, t10, btn);

        // 1 saniye beklerken RX dinle
        // Yaklaşık 20 x 50ms = 1000ms
        for(i=0; i<20; i++){
            uart0_process_rx();
            delay_ms(50);
        }

        // 1 saniye geçti -> saati ilerlet
        clock_tick_1s();
    }
}














