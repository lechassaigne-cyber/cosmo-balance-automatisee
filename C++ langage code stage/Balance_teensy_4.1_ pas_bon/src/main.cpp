#include <Arduino.h>
#include "HX711.h"

#define DOUT_1 2
#define SCK_1  3

HX711 scale;

const long OFFSET = 292255;
const float FACTEUR = 1673.85;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    scale.begin(DOUT_1, SCK_1);

    Serial.println("Lecture en grammes");
}

void loop()
{
    long raw = scale.read();

    float poids = (raw - OFFSET) / FACTEUR;

    Serial.print("Poids : ");
    Serial.print(poids, 2);
    Serial.println(" g");

    delay(500);
}