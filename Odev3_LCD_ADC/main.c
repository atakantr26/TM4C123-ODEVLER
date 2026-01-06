#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// --- PIN EŞLEME ---
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

// --- GECIKME ---
static inline void delay_us(uint32_t us) {
    SysCtlDelay((SysCtlClockGet() / 3000000) * us); // ~1us @80MHz
}
static inline void delay_ms(uint32_t ms){
    while(ms--) delay_us(1000);
}

// --- LCD TEMEL ---
static void pulseE(void){
    GPIOPinWrite(LCD_PORT_E_BASE, LCD_PIN_E, LCD_PIN_E);
    delay_us(2);
    GPIOPinWrite(LCD_PORT_E_BASE, LCD_PIN_E, 0);
    delay_us(50);
}
static void write4(uint8_t nibble){
    GPIOPinWrite(LCD_PORT_D_BASE, LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7, 0);
    uint8_t out = 0;
    if(nibble & 0x1) out |= LCD_PIN_D4;
    if(nibble & 0x2) out |= LCD_PIN_D5;
    if(nibble & 0x4) out |= LCD_PIN_D6;
    if(nibble & 0x8) out |= LCD_PIN_D7;
    GPIOPinWrite(LCD_PORT_D_BASE, LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7, out);
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
static void lcd_puts(const char *s){ while(*s) putc_lcd(*s++); }

static void lcd_goto(uint8_t col, uint8_t row){
    const uint8_t offsets[] = {0x00, 0x40};
    cmd(0x80 | (col + offsets[row]));
}

static void lcd_init(void){
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
    GPIOPinTypeGPIOOutput(LCD_PORT_D_BASE,  LCD_PIN_D4|LCD_PIN_D5|LCD_PIN_D6|LCD_PIN_D7);

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

// --- SAYI YAZDIRMA (genel amaçlı, işaretli) ---
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

// --- SAAT YARDIMCILARI ---
static void lcd_put2(uint8_t v){
    putc_lcd('0' + (v/10));
    putc_lcd('0' + (v%10));
}

// SAAT: üst satırda (satır 0, kolondan 5 sonrası)
static void time_print(uint8_t h, uint8_t m, uint8_t s){
    lcd_goto(5,0);      // ÜST SATIR
    lcd_put2(h); putc_lcd(':');
    lcd_put2(m); putc_lcd(':');
    lcd_put2(s);
}

// 24:59:59 → 00:00:00
static void time_tick(uint8_t *h, uint8_t *m, uint8_t *s){
    if(++(*s) >= 60){
        *s = 0;
        if(++(*m) >= 60){
            *m = 0;
            if(++(*h) >= 24){
                *h = 0;
            }
        }
    }

    // Ek koruma (artık çok da gerek yok ama kalsın)
    if(*h >= 23 && *m >= 59 && *s >= 59){
        *h = 0; *m = 0; *s = 0;
    }
}

// --- ADC: Dahili SICAKLIK SENSÖRÜ (ADC0, TS) ---
static void adc_init_temp(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    // Daha stabil ölçüm için oversample
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // ADC_CTL_TS: dahili sıcaklık sensörü
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

// TM4C123 formülü (yaklaşık):
// Temp(C) = 147.5 - (75 * Vref * ADC / 4096)
// Vref=3.3 -> 75*3.3=247.5
// Biz 10x ile çalışalım: Temp10 = 1475 - (2475 * ADC / 4096)
static int32_t temp_raw_to_c10(uint32_t adcVal){
    int32_t t10 = 1475 - ( (2475 * (int32_t)adcVal) / 4096 );
    return t10; // 10x C (ör: 253 -> 25.3°C)
}

// ALT SATIR: "S: xx.xC"
static void temp_print(int32_t t10){
    int32_t t_int  = t10 / 10;
    int32_t t_frac = t10 % 10;
    if(t_frac < 0) t_frac = -t_frac;

    lcd_goto(3,1);      // "S:" den sonrası
    lcd_put_int(t_int);
    putc_lcd('.');
    putc_lcd('0' + (int)t_frac);
    putc_lcd('C');
    lcd_puts("   ");    // eski karakter silmek için boşluk
}

// ===================== MAIN =====================
int main(void){
    lcd_init();
    adc_init_temp();

    // Başlangıç SAAT ve SICAKLIK başlıkları
    lcd_goto(0,0); lcd_puts("SAAT:");
    lcd_goto(0,1); lcd_puts("S:");

    // Başlangıç saat:
    uint8_t hh = 23;
    uint8_t mm = 59;
    uint8_t ss = 50;

    time_print(hh, mm, ss);

    while(1){
        delay_ms(1000);          // 1 saniye bekle
        time_tick(&hh, &mm, &ss);
        time_print(hh, mm, ss);  // ÜST SATIRDAN SAAT

        // SICAKLIK OKU ve ALT SATIRA YAZ
        uint32_t raw   = adc_read_temp_raw();
        int32_t  t10   = temp_raw_to_c10(raw);
        temp_print(t10);
    }
}



