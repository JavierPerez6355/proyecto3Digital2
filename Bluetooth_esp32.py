#include <Arduino.h>

#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#include <DabbleESP32.h>

#define RX_NUCLEO 16
#define TX_NUCLEO 17

String respuestaNucleo = "";

void setup()
{
    // Comunicación con el monitor serial de la compu
    Serial.begin(115200);

    // Comunicación UART con Nucleo 1
    Serial2.begin(115200, SERIAL_8N1, RX_NUCLEO,TX_NUCLEO);

    Dabble.begin("FICESP32");

    respuestaNucleo.reserve(128);

    Serial.println("ESP32 lista");
    Serial.println("Conecta Dabble");
}

void loop()
{
    // Atender los mensajes Bluetooth de Dabble
    Dabble.processInput();

    // Telefono a ESP32 a Nucleo 1
    if (Terminal.available() > 0)
    {
        char ultimoCaracter = '\0';

        while (Terminal.available() > 0)
        {
            char caracter = (char)Terminal.read();

            Serial2.write((uint8_t)caracter);
            Serial.write((uint8_t)caracter);

            ultimoCaracter = caracter;
        }

        // Agregar el final de linea que espera Nucleo 1
        if (ultimoCaracter != '\n')
        {
            Serial2.write('\n');
            Serial.println();
        }
    }

    // Permitir pruebas desde el monitor serial por USB
    while (Serial.available() > 0)
    {
        Serial2.write((uint8_t)Serial.read());
    }

    // Nucleo 1 a ESP32 a telefono y computadora
    while (Serial2.available() > 0)
    {
        char caracter = (char)Serial2.read();
        Serial.write((uint8_t)caracter);
        respuestaNucleo += caracter;

        // Enviar al telefono al completar una linea
        // El limite evita acumular texto indefinidamente
        if (caracter == '\n' || respuestaNucleo.length() >= 120)
        {
            Terminal.print(respuestaNucleo);
            respuestaNucleo = "";
        }
    }
}