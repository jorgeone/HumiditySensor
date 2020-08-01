#include "SigFox.h"
#include "ArduinoLowPower.h"
#include "DHT.h"


#define DHTPIN        1                // Introducimos el pin de la entrada
#define DHTTYPE       DHT11
#define DEBUG         true             
#define SLEEPTIME     1 * 60 * 1000   // Programamos el tiempo con el que enviará la señal, 1 min (1 min x 60 segundos x 1000 milisegundos)

#define UINT16_t_MAX  65536
#define INT16_t_MAX   UINT16_t_MAX/2

typedef struct __attribute__ ((packed)) sigfox_message {
        int16_t moduleTemperature;
        int16_t dhtTemperature;
        uint16_t dhtHumidity;
        uint8_t lastMessageStatus;
} SigfoxMessage;

SigfoxMessage msg;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
        if (DEBUG) {
                Serial1.begin(115200);
                while (!Serial1) {}
        }

        if (!SigFox.begin()) {
                // Si algo no funciona correctamente reiniciaremos porque la placa puede haberse quedado bloqueada
                reboot();
        }

        // Enviamos el módulo al modo de espera hasta que necesitemos enviar el mensaje
        SigFox.end();

        if (DEBUG) {
                SigFox.debug();
        }

        dht.begin();
}

void loop() {
        // Leer la humedad toma alrededor de 250 milisegundos!
        // Las lecturas del sensor también pueden tardar hasta 2 segundos (Porque es un sensor muy lento)
        float h = dht.readHumidity();
        // Leemos la temperatura en grados Celsius (por defecto)
        float t = dht.readTemperature();
        // Comprobamos si hay algun fallo con el sensor (Para intentarlo de nuevo)
        if (isnan(h) || isnan(t)) {
                Serial.println("Fallo al leer el sensor!");
                return;
        }

        msg.dhtTemperature = convertoFloatToInt16(t, 60, -60);
        msg.dhtHumidity = convertoFloatToUInt16(h, 110);

        if(DEBUG) {
                Serial.print("Humidity: ");
                Serial.print(h);
                Serial.print(" %\t");
                Serial.print("Temperature: ");
                Serial.print(t);
                Serial.print(" *C ");
        }
        // Iniciamos el módulo
        SigFox.begin();
        // Esperamos 30 ms después de la primera configuración (100 ms antes)
        delay(100);

        t = SigFox.internalTemperature();
        msg.moduleTemperature = convertoFloatToInt16(t, 60, -60);

        // Borramos todas las interrupciones pendientes
        SigFox.status();
        delay(1);

        SigFox.beginPacket();
        SigFox.write((uint8_t*)&msg, 12);

        msg.lastMessageStatus = SigFox.endPacket();

        SigFox.end();
        //Introducimos aquí el sleep del principio
        LowPower.sleep(SLEEPTIME);
}

void reboot() {
        NVIC_SystemReset();
        while (1) ;
}

int16_t convertoFloatToInt16(float value, long max, long min) {
        float conversionFactor = (float) (INT16_t_MAX) / (float)(max - min);
        return (int16_t)(value * conversionFactor);
}

uint16_t convertoFloatToUInt16(float value, long max) {
        float conversionFactor = (float) (UINT16_t_MAX) / (float)(max);
        return (uint16_t)(value * conversionFactor);
}
