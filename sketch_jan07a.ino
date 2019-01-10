/*----------------------------------------------------------------------------*
*-----------------------------------------------------------------------------*
*--------------------------------[VERSION 1.1]--------------------------------*
*-----------------------------------------------------------------------------*
*-----------------------------------------------------------------------------*/

#define F_CPU           16000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define LIGHT_BORDER    810     // Граница освещенности комнаты, при которой включается подсветка;
#define MAX_TIME        60000   // Таймер;
#define MAX_HUM         75.0    // Граница влажности в комнате;

#define RELAY_FAN       6       // DDRD;
#define RELAY_LED       7       // DDRD;
#define MOVSENSOR       0       // DDRB;
#define LED_ONBOARD     5       // DDRB;
#define LIGHTSENSOR     3       // DDRC;
#define DHTPIN          2       // DDRC;


float h = 0.0;

unsigned long currentTime = 0;  // для хранения информации о текущем времени
bool FLAG = false;              // ключевой флаг

int result;                     // хранение результата полученного с АЦП
uint8_t analog_ref = DEFAULT;   // хранение предыдущего состояния ADMUX

DHT dht(A2, DHT11);             // инициализация экземпляра для работы с DHT

void setup()
{
    wdt_disable();              // отключение watchdog для инициализации

    dht.begin();                // инициализация DHT датчика

    FLAG = false;               // обнуление ключевого флага

    DDRD |= (1 << RELAY_FAN);   // настройка D6 на выход
    DDRD |= (1 << RELAY_LED);   // настройка D7 на выход

    DDRB &= ~(1 << MOVSENSOR);  // настройка D8 на вход
    DDRB |= (1 << LED_ONBOARD); // настройка D13 на выход

    DDRC &= ~(1 << LIGHTSENSOR);// настройка А3 на вход
    DDRC &= ~(1 << DHTPIN);     // настройка А2 на вход

    PORTD |= (1 << RELAY_FAN);  // устанавливаем HIGH на D6
    PORTD |= (1 << RELAY_LED);  // устанавливаем HIGH на D7

    PORTB &= ~(1 << LED_ONBOARD);// отключение светодиода на плате

    wdt_enable(WDTO_8S);        // настройка watchdog
}

void loop()
{
    // Если прошло больше минуты...
    if ((millis() - currentTime >= MAX_TIME) && FLAG == true) 
    {
        FLAG = false; // обнуляем флаг
        PORTD |= (1 << RELAY_FAN); // устанавливаем HIGH на D6
        PORTD |= (1 << RELAY_LED); // устанавливаем HIGH на D7
    } 
    _analogRead(); // получаем данные с АЦП
    _delay_ms(100); // задержка для получения данных
    if (bitRead(PINB, 0)) // if moverment detected...
    {        
        PORTD &= ~(1 << RELAY_FAN); // априорно включаем вентилятор
        if (result >= LIGHT_BORDER) // проверяем границу освещенности
        {
            // если в комнате темно включаем подсветку
            PORTD &= ~(1 << RELAY_LED);
        }
        else PORTD |= (1 << RELAY_LED);
        currentTime = millis(); // запоминаем текущее время
        FLAG = true; // поднимаем флаг
        wdt_reset();
        return;
    }

    h = dht.readHumidity(); // считываем влажность с DHT датчика

    if (isnan(h)) // проверка на корректность данных
    {
        PORTB |= (1 << LED_ONBOARD); // включаем сигнализирующий светодиод на плате
        wdt_reset(); // сбрасываем watchdog
        return;
    }
    // Если текущая влажность или температура выше установленной границы...
    if (h >= MAX_HUM) 
    { 
        PORTD &= ~(1 << RELAY_FAN); // включаем вентилятор
        currentTime = millis(); // запоминаем текущее время
        FLAG = true; // поднимаем флаг
    }

    wdt_reset(); // сбрасываем watchdog
}
void _analogRead()
{
  ADCSRA = 0;             // Сбрасываем регистр ADCSRA
  ADCSRB = 0;             // Сбрасываем регистр ADCSRB
  ADMUX |= (1 << REFS0);  // Задаем ИОН
  
  ADMUX |= (1 << ADLAR);  // Меняем порядок записи бит, чтобы можно было читать только 8 бит регистра ADCH
                          // Таким образом отсекаются 2 последних "шумных" бита, результат 8-битный (0...255)
  
  analog_ref = ADMUX;     // Запоминаем состояние регистра - оно будет использоваться при смене пина входящего сигнала

  ADMUX |= (3 & 0x07);    // Выбираем пин A3 для преобразования
  // Устанавливаем предделитель - 16 (ADPS[2:0]=100)
  ADCSRA |= (1 << ADPS2);                     //Биту ADPS2 присваиваем единицу
  ADCSRA &= ~ ((1 << ADPS1) | (1 << ADPS0));  //Битам ADPS1 и ADPS0 присваиваем нули

  ADCSRA |= (1 << ADATE); // Включаем автоматическое преобразование
  ADCSRA |= (1 << ADIE);  // Разрешаем прерывания по завершении преобразования
  ADCSRA |= (1 << ADEN);  // Включаем АЦП
  ADCSRA |= (1 << ADSC);  // Запускаем преобразование
}
ISR(ADC_vect)
{
  //...
  // Порядок записи результирующих битов был изменен из-за пренебрежения последними битами
  // ADLAR=1
  result = ADCH;  // Считываем только значимые 8 бит - значение из диапазона 0...255
  // Если ADLAR=1:
  result = (ADCL>> 6) | (ADCH << 2);
}