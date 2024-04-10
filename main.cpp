#include <stdio.h>
#include <string.h>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"
#include "uart/PicoUart.h"

#include "IPStack.h"
#include "Countdown.h"
#include "MQTTClient.h"
#include "ModbusClient.h"
#include "ModbusRegister.h"
#include "ssd1306.h"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#if 0
#define UART_NR 0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#else
#define UART_NR 1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define ROT_A_PIN 10
#define ROT_B_PIN 11
#define ROT_SW_PIN 12
int initial_speed = 0;  // Initial speed 0% (0-1000)
uint8_t auto_pressure = 0; // for automatic mode
int auto_mode = 0; // flag, 1 or 0

#endif

#define BAUD_RATE 9600
#define STOP_BITS 1 // for simulator
//#define STOP_BITS 2 // for real system

#define USE_MODBUS
#define USE_MQTT
#define USE_SSD1306

#define SCALE_FACTOR 240
#define ALTITUDE_CORR_FACTOR 0.95

#ifdef USE_SSD1306
static const uint8_t raspberry26x32[] =
        {0x0, 0x0, 0xe, 0x7e, 0xfe, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xf8, 0xfc, 0xfe,
         0xfe, 0xff, 0xff,0xff, 0xff, 0xff, 0xfe, 0x7e,
         0x1e, 0x0, 0x0, 0x0, 0x80, 0xe0, 0xf8, 0xfd,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd,
         0xf8, 0xe0, 0x80, 0x0, 0x0, 0x1e, 0x7f, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0x7f, 0x1e, 0x0, 0x0,
         0x0, 0x3, 0x7, 0xf, 0x1f, 0x1f, 0x3f, 0x3f,
         0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f,
         0x3f, 0x1f, 0x1f, 0xf, 0x7, 0x3, 0x0, 0x0 };

static const uint8_t ventilator40x40[] =
        {
                0xff, 0x7f, 0x0f, 0x07, 0xc3, 0xe3, 0xf3, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0x71,
                0x71, 0x71, 0x71, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1,
                0xf1, 0xf3, 0xe3, 0xc3, 0x07, 0x0f, 0x3f, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xc7, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0x3f,
                0x1f, 0x0f, 0x0f, 0x07, 0x07, 0x0f, 0x0f, 0x1f, 0x3f, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff,
                0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x1f, 0x0f, 0x0f, 0x0f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1c,
                0x1c, 0x00, 0x80, 0x91, 0x91, 0x03, 0x00, 0x38, 0x38, 0x78, 0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0,
                0xf0, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xfc,
                0xf8, 0xf0, 0xe0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0x1f, 0x06, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x81, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff,
                0xff, 0xfe, 0xf0, 0xe0, 0xc3, 0xc7, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f,
                0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8e, 0x8e, 0x8e, 0x8e, 0x8e, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f,
                0x8f, 0x8f, 0xc7, 0xc3, 0xe0, 0xf0, 0xfc, 0xff

        };
#endif

//EEPROM
#define DEVICE_ADDR 0x50
#define EEPROM_SIZE 0x8000
#define MEMORY_I2C i2c0

uint16_t auto_addr_inverted = EEPROM_SIZE - 10;
uint16_t auto_addr = EEPROM_SIZE - 20;
uint16_t pressure_addr = EEPROM_SIZE - 30;
uint16_t speed_addr = EEPROM_SIZE - 40;
void write_memory(uint16_t memory_address, uint8_t data);
uint8_t read_memory(uint16_t memory_address);
void loadFromMemory() {
    uint8_t isAuto = read_memory(auto_addr);
    uint8_t isNotAuto = read_memory(auto_addr_inverted);
    auto_mode = (isAuto != isNotAuto) ? isAuto : 0;
    auto_pressure = read_memory(pressure_addr);
    initial_speed = read_memory(speed_addr) * 10; // we store only the percent, so it fits in a byte
}

void saveStatesToMemory() {
    write_memory(auto_addr, auto_mode);
    write_memory(auto_addr_inverted, !auto_mode);
    write_memory(speed_addr, (int)(initial_speed/10));
    write_memory(pressure_addr, auto_pressure);
}

void write_memory(uint16_t memory_address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    buffer[2] = data;
    i2c_write_blocking(MEMORY_I2C, DEVICE_ADDR, buffer, 3, false);
    sleep_ms(5);
}

uint8_t read_memory(uint16_t memory_address) {
    uint8_t value;
    uint8_t buffer[2];
    buffer[0] = memory_address >> 8;
    buffer[1] = memory_address & 0xFF;
    i2c_write_blocking(MEMORY_I2C, DEVICE_ADDR, buffer, 2, false);
    sleep_ms(5);
    i2c_read_blocking(MEMORY_I2C, DEVICE_ADDR, &value, 1, false);
    return value;
}
void messageArrived(MQTT::MessageData &md) {
    MQTT::Message &message = md.message;
    char buffer[100];//28 when false, 30 when true
    printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\n",
           message.qos, message.retained, message.dup, message.id);
    printf("Payload %s\n", (char *) message.payload);
    char * payload = (char*) message.payload;
    if (payload[0] == '{') {
        printf("=======================\n");
        for (int i = 0; i< 100; i++) {
            char current_char = payload[i];
            buffer[i] = payload[i];
            if (current_char == '}') {
                break;
            }
        }
        //If auto mode, set pressure with web-UI messages
        if (buffer[2] == 'a' && buffer[3] == 'u' && buffer[4] == 't' && buffer[5] == 'o') {
            if (buffer[9] == 't') {
                auto_mode = 1;
                if (buffer[28] == '}') {
                    auto_pressure = ((int)buffer[27]-48);
                } else if (buffer[29] == '}') {
                    auto_pressure = ((int)buffer[27]-48)*10 + ((int)buffer[28]-48); // *10 for scaling to web-UI
                } else if (buffer[30] == '}') {
                    auto_pressure = ((int)buffer[27]-48)*100 + ((int)buffer[28]-48)*10 + ((int)buffer[29]-48);
                }
            }
            if (buffer[9] == 'f'){
                auto_mode = 0;
                if (buffer[26] == '}') {
                    initial_speed = ((int)buffer[25]-48)*10;
                } else if (buffer[27] == '}') {
                    initial_speed = ((int)buffer[25]-48)*100 + ((int)buffer[26]-48)*10;
                } else if (buffer[28] == '}') {
                    initial_speed = 1000;
                }
            }
        }
    }
}
// Interrupt handler for rotary encoder
static void a_interrupt_handler(uint gpio, uint32_t event) {
    if(gpio == ROT_A_PIN && gpio_get(ROT_B_PIN)){
        initial_speed -= 25;
        if (initial_speed <= 0) {
            initial_speed = 0;
        }
    }else{
        initial_speed += 25;
        if (initial_speed >= 1000) {
            initial_speed = 1000;
        }
    }
}

static const char *topic = "g09/controller/settings"; //topics
static const char *topic1 = "g09/controller/status"; //topics

uint16_t pressure_measure() {
    uint8_t pressure_read[2];
    uint8_t start[] = {0xF1};
    i2c_write_blocking(i2c1, 0x40, start, 1, false);
    sleep_ms(10);
    i2c_read_blocking(i2c1, 0x40, pressure_read, 2, false);
    sleep_ms(100);
    uint16_t measurement = ((pressure_read[0] << 8) | pressure_read[1]) / SCALE_FACTOR * ALTITUDE_CORR_FACTOR;
    if (measurement>140){
        measurement = 0; //Hard fixed for unexpected values of pressure greater than 140, for example 250
    }
    return measurement;
}

int main() {
    std::vector<int> readings;
    const uint led_pin = 22;
    const uint button = 9;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);

    //initialize rotary encoder
    gpio_init(ROT_A_PIN);
    gpio_init(ROT_B_PIN);
    gpio_init(ROT_SW_PIN);

    gpio_set_dir(ROT_A_PIN, GPIO_IN);
    gpio_set_dir(ROT_B_PIN, GPIO_IN);
    gpio_set_dir(ROT_SW_PIN, GPIO_IN);
    gpio_set_pulls(ROT_SW_PIN, true, false);  // Enable pull-up for the switch

    gpio_set_irq_enabled_with_callback(ROT_A_PIN, GPIO_IRQ_EDGE_RISE, true, &a_interrupt_handler);

    // Initialize chosen serial port
    stdio_init_all();

    printf("\nBoot\n");
#ifdef USE_SSD1306
    // I2C is "open drain",
    // pull-ups to keep signal high when no data is being sent
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C); // the display has external pull-ups
    gpio_set_function(15, GPIO_FUNC_I2C); // the display has external pull-ups
    i2c_init(i2c0, 100 * 1000); // EEPROM
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);
    ssd1306 display(i2c1);
    display.fill(0);
    display.text("Welcome!", 0, 0);
    mono_vlsb rb(ventilator40x40, 40, 40);
    display.blit(rb, 2, 20);
    display.text("Credits:", 45, 20, 1);
    display.text(">Andrea", 50, 30, 1);
    display.text(">Daniel", 50, 40, 1);
    display.text(">Jeferson", 50, 50, 1);
    //display.rect(15, 15, 35, 45, 1);
    //display.line(60, 5, 120, 60, 1);
    //display.line(60, 60, 120, 5, 1);
    //Write a welcome message
    display.show();
    loadFromMemory();
#if 0
    for(int i = 0; i < 128; ++i) {
        sleep_ms(20);
        display.scroll(1, 0);
        display.show();
    }
    display.text("Done", 20, 20);
    display.show();
#endif

#endif
    uint16_t pressure_measurement = 0;

#ifdef USE_MQTT
    //IPStack ipstack("SSID", "PASSWORD"); // example
    IPStack ipstack("SmartIotMQTT", "SmartIot"); // example
    auto client = MQTT::Client<IPStack, Countdown, 250>(ipstack); //added 200, for max size of the MQTT-mssg

    int rc = ipstack.connect("192.168.1.10", 1883); // mqtt server ip and port, SmartIotMQTT server IP: 192.168.1.10
    if (rc != 1) {
        printf("rc from TCP connect is %d\n", rc);
    }
    printf("MQTT connecting\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *) "PicoW-sample";
    rc = client.connect(data);
    if (rc != 0) {
        printf("rc from MQTT connect is %d\n", rc);
        while (true) {
            tight_loop_contents();
        }
    }
    printf("MQTT connected\n");

    // We subscribe QoS2. Messages sent with lower QoS will be delivered using the QoS they were sent with
    rc = client.subscribe(topic, MQTT::QOS2, messageArrived);
    if (rc != 0) {
        printf("rc from MQTT subscribe is %d\n", rc);
    }
    printf("MQTT subscribed\n");

    auto mqtt_send = make_timeout_time_ms(2000);
    int mqtt_qos = 0;
    int msg_count = 0;
#endif

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE, STOP_BITS)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister rh(rtu_client, 241, 256);
    ModbusRegister temp(rtu_client, 241, 257);
    ModbusRegister co2(rtu_client, 240, 256);
    auto modbus_poll = make_timeout_time_ms(1500);
    auto display_poll = make_timeout_time_ms(10);
    ModbusRegister produal(rtu_client, 1, 0); // fan
    //display.fill(0);
    auto temperature = static_cast<float>(temp.read()) / 10.0;
    auto r_humidity = static_cast<float>(rh.read()) / 10.0;
    auto carbon_dioxide = static_cast<float>(co2.read()); //static_cast<float>(co2.read())
#endif

    while (true) {
#ifdef USE_MODBUS
        if (time_reached(modbus_poll)) {
            gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
            modbus_poll = delayed_by_ms(modbus_poll, 3000);
            temperature = static_cast<float>(temp.read()) / 10.0;
            r_humidity = static_cast<float>(rh.read()) / 10.0;
            carbon_dioxide = static_cast<float>(co2.read());
            printf("RH=%5.1f%%\n", r_humidity);
            printf("TEMP=%5.1f C\n", temperature);
            printf("CO2=%5.1f ppm\n", carbon_dioxide);
            pressure_measurement = pressure_measure();
            if (auto_mode==1 && readings.size() < 20) readings.push_back(pressure_measurement);
            else if (auto_mode == 1 && readings.size() == 20) {
                readings.erase(readings.begin());
                readings.push_back(pressure_measurement);
            }else(readings.clear());//if
            printf("Pressure=%u \n", pressure_measurement);
            printf("Fan speed=%d \n", initial_speed / 10);
            printf("AUTO mode=%d \n", auto_mode);
            if (auto_mode) {
                if (pressure_measurement < auto_pressure - 1) {
                    initial_speed += (auto_pressure-pressure_measurement)*6; //*6
                }
                if (pressure_measurement > auto_pressure + 1) {
                    initial_speed -= (pressure_measurement-auto_pressure)*6; //*6
                }
                if (initial_speed<0)
                    initial_speed = 0;
                if (initial_speed>1000)
                    initial_speed = 1000;
            }
            saveStatesToMemory();
        }
        produal.write(initial_speed);

        //Show data on display
        if (time_reached(display_poll)) {
            display_poll = delayed_by_ms(display_poll, 10);
            display.fill(0);
            //Slide bar for fan speed
            display.rect(0, 0, int(initial_speed*0.128),5, 1,true);

            //Measurements
            char tempString[10];
            char humString[10];
            char co2String[10];
            char pressureString[10];
            char fanspeedString[20];
            char error[24];
            sprintf(tempString, "T:%.1f", temperature);
            sprintf(humString, "RH :%.1f", r_humidity);
            sprintf(co2String, "CO2:%.1f", carbon_dioxide);
            sprintf(pressureString, "P:%u", pressure_measurement);
            sprintf(fanspeedString, "Fan Speed:%d%%", initial_speed / 10);
            sprintf(error,"P not set in 1m");
            display.text(tempString, 0, 30, 1);
            display.text(humString, 55, 30, 1);
            display.text(co2String, 55, 40, 1);
            display.text(pressureString, 0, 40, 1);
            if (auto_mode == 1 && (!(auto_pressure < (*readings.end()-5) || auto_pressure  > (*readings.end()+5)))){
                display.rect(0,49,128,10,1, true);
                display.text(error, 0, 50, 0);
            }
            if(auto_mode){
                display.text("[AUTO]", 45, 15, 1);
            } else{
                display.text(fanspeedString, 10, 15, 1);
            }
            display.show();
        }
#endif
#ifdef USE_MQTT
        if (time_reached(mqtt_send)) {
            mqtt_send = delayed_by_ms(mqtt_send, 2000);
            if (!client.isConnected()) {
                printf("Not connected...\n");
                rc = client.connect(data);
                if (rc != 0) {
                    printf("rc from MQTT connect is %d\n", rc);
                }
            }
            char buf[250]; //original buffer 100
            //int rc = 0;
            MQTT::Message message{};
            message.retained = false;
            message.dup = false;
            message.payload = (void *) buf;
            switch (mqtt_qos) {
                case 0:
                    // Send and receive QoS 0 message
                    //sprintf(buf, "Msg nr: %d QoS 1 message_send", ++msg_count);
                    sprintf(buf,"{\"pressure\": %u, "
                                "\"auto\": %s, "
                                "\"speed\": %d, "
                                "\"co2\": %.1f, "
                                "\"rh\": %.1f, "
                                "\"temp\": %.1f, "
                                "\"setpoint\": %d, "
                                "\"error\": %s, "
                                "\"nr\": %d "
                                "}"
                            ,pressure_measurement,
                            (auto_mode == 1) ? "true" : "false",
                            initial_speed / 10,
                            carbon_dioxide,
                            r_humidity,
                            temperature,
                            (auto_mode == 1) ? auto_pressure : (initial_speed / 10),
                            (auto_mode == 1 && (!(auto_pressure < (*readings.end()-5) || auto_pressure  > (*readings.end()+5))) &&readings.size()==20) ? "true" : "false",
                            ++msg_count);
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS0;
                    message.payloadlen = strlen(buf);
                    rc = client.publish(topic1, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
                case 1:
                    // Send and receive QoS 1 message
                    //sprintf(buf, "Msg nr: %d QoS 1 message_send", ++msg_count);
                    sprintf(buf,"{\"pressure\": %u, "
                                "\"auto\": %s, "
                                "\"speed\": %d, "
                                "\"co2\": %.1f, "
                                "\"rh\": %.1f, "
                                "\"temp\": %.1f, "
                                "\"setpoint\": %d, "
                                "\"error\": %s, "
                                "\"nr\": %d "
                                "}"
                            ,pressure_measurement,
                            (auto_mode == 1) ? "true" : "false",
                            initial_speed / 10,
                            carbon_dioxide,
                            r_humidity,
                            temperature,
                            (auto_mode == 1) ? auto_pressure : (initial_speed / 10),
                            (auto_mode == 1 && (!(auto_pressure < (*readings.end()-5) || auto_pressure  > (*readings.end()+5))) &&readings.size()==20) ? "true" : "false",
                            ++msg_count);
                    printf("%s\n", buf);
                    message.qos = MQTT::QOS1;
                    message.payloadlen = strlen(buf);
                    rc = client.publish(topic1, message);
                    printf("Publish rc=%d\n", rc);
                    ++mqtt_qos;
                    break;
#if MQTTCLIENT_QOS2
                    case 2:
                        // Send and receive QoS 2 message
                        sprintf(buf, "Msg nr: %d QoS 2 message", ++msg_count);
                        printf("%s\n", buf);
                        message.qos = MQTT::QOS2;
                        message.payloadlen = strlen(buf) + 1;
                        rc = client.publish(topic, message);
                        printf("Publish rc=%d\n", rc);
                        ++mqtt_qos;
                        break;
#endif
                default:
                    mqtt_qos = 0;
                    break;
            }
        }
        cyw43_arch_poll(); // obsolete? - see below
        client.yield(100); // socket that client uses calls cyw43_arch_poll()
#endif
    }
}
