#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

#define SDA_PIN 23
#define SCL_PIN 22

#define PCA_ADDR 0x70

#define CH1 1
#define CH5 5
#define CH7 7

#define TARE_TIME 5000
#define CAL_TIME 5000
#define CAL_WEIGHT 150.0

NAU7802 nau1;
NAU7802 nau5;
NAU7802 nau7;

long zero1 = 0;
long zero5 = 0;
long zero7 = 0;

float factor1 = 1.0;
float factor5 = 1.0;
float factor7 = 1.0;




void selectChannel(uint8_t ch)
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();

    delay(10);
}



bool initNAU(NAU7802 &nau, uint8_t ch)
{
    selectChannel(ch);

    if (!nau.begin(Wire))
    {
        Serial.printf("CH%d INIT FAILED\n", ch);
        return false;
    }

    nau.setGain(NAU7802_GAIN_128);
    nau.setSampleRate(NAU7802_SPS_80);

    if (!nau.calibrateAFE())
    {
        Serial.printf("CH%d AFE CAL FAILED\n", ch);
        return false;
    }

    Serial.printf("CH%d READY\n", ch);

    return true;
}


// =====================================================
// READ SENSOR
// =====================================================

bool readSensor(
    NAU7802 &nau,
    uint8_t ch,
    long &value)
{
    selectChannel(ch);

    unsigned long start = millis();

    while (!nau.available())
    {
        if (millis() - start > 100)
            return false;

        delay(1);
    }

    value = nau.getReading();

    return true;
}



void tare() // TARE IT  for all
{
    Serial.println();
    Serial.println("================================");
    Serial.println("TARE CH1 / CH5 / CH7");
    Serial.println("================================");

    Serial.println("REMOVE ALL WEIGHT.");
    Serial.println("Starting in 3 seconds...");

    delay(3000);

    long long sum1 = 0;
    long long sum5 = 0;
    long long sum7 = 0;

    long count1 = 0;
    long count5 = 0;
    long count7 = 0;

    unsigned long start = millis();

    Serial.println("Taring for 5 seconds...");

    while (millis() - start < TARE_TIME)
    {
        long value;

        // CH1
        if (readSensor(nau1, CH1, value))
        {
            sum1 += value;
            count1++;
        }

        // CH5
        if (readSensor(nau5, CH5, value))
        {
            sum5 += value;
            count5++;
        }

        // CH7
        if (readSensor(nau7, CH7, value))
        {
            sum7 += value;
            count7++;
        }
    }

    if (count1 > 0)
        zero1 = sum1 / count1;

    if (count5 > 0)
        zero5 = sum5 / count5;

    if (count7 > 0)
        zero7 = sum7 / count7;

    Serial.println();
    Serial.println("TARE COMPLETE");

    Serial.print("CH1 ZERO = ");
    Serial.println(zero1);

    Serial.print("CH5 ZERO = ");
    Serial.println(zero5);

    Serial.print("CH7 ZERO = ");
    Serial.println(zero7);
}




float calibrateChannel(
    NAU7802 &nau,
    uint8_t ch,
    long zero)
{
    Serial.println();
    Serial.printf(
        "Place %.0f g on CH%d\n",
        CAL_WEIGHT,
        ch
    );

    Serial.println("Starting in 3 seconds...");

    delay(3000);

    long long sum = 0;
    long count = 0;

    unsigned long start = millis();

    while (millis() - start < CAL_TIME)
    {
        long value;

        if (readSensor(nau, ch, value))
        {
            sum += value;
            count++;
        }
    }

    if (count == 0)
    {
        Serial.printf(
            "CH%d CALIBRATION FAILED\n",
            ch
        );

        return 1.0;
    }

    long average = sum / count;

    float factor =
        ((float)(average - zero)) / CAL_WEIGHT;

    Serial.printf(
        "CH%d CAL RAW = %ld\n",
        ch,
        average
    );

    Serial.printf(
        "CH%d FACTOR = %.6f counts/g\n",
        ch,
        factor
    );

    return factor;
}



void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    Wire.setClock(100000);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" ESP32 + PCA9548A");
    Serial.println(" 3x NAU7802");
    Serial.println(" CH1 / CH5 / CH7");
    Serial.println("================================");



    if (!initNAU(nau1, CH1))
        while (1);

    if (!initNAU(nau5, CH5))
        while (1);


if (!initNAU(nau7, CH7))
        while (1);


    tare();



    factor1 =
        calibrateChannel(
            nau1,
            CH1,
            zero1
        );

    Serial.println();
    Serial.println("Remove 150 g from CH1.");
    delay(3000);


    factor5 =
        calibrateChannel(
            nau5,
            CH5,
            zero5
        );

    Serial.println();
    Serial.println("Remove 150 g from CH5.");
    delay(3000);


    factor7 =
        calibrateChannel(
            nau7,
            CH7,
            zero7
        );

    Serial.println();
    Serial.println("Remove 150 g from CH7.");
    delay(3000);



    Serial.println();
    Serial.println("================================");
    Serial.println("CALIBRATION COMPLETE");
    Serial.println("================================");

    Serial.printf(
        "CH1 factor = %.6f counts/g\n",
        factor1
    );

    Serial.printf(
        "CH5 factor = %.6f counts/g\n",
        factor5
    );

    Serial.printf(
        "CH7 factor = %.6f counts/g\n",
        factor7
    );

    Serial.println();
    Serial.println("Starting weight measurement...");
    Serial.println();
}


void loop()
{
    long raw1;
    long raw5;
    long raw7;

    bool ok1 =
        readSensor(
            nau1,
            CH1,
            raw1
        );

    bool ok5 =
        readSensor(
            nau5,
            CH5,
            raw5
        );

    bool ok7 =
        readSensor(
            nau7,
            CH7,
            raw7
        );


    if (ok1)
    {
        float grams1 =
            (raw1 - zero1) / factor1;

        if (grams1 < 0)
            grams1 = 0;

        Serial.print("CH1: ");
        Serial.print(grams1, 2);
        Serial.print(" g");
    }
    else
    {
        Serial.print("CH1: --");
    }


    Serial.print("    ");



    if (ok5)
    {
        float grams5 =
            (raw5 - zero5) / factor5;

        if (grams5 < 0)
            grams5 = 0;

        Serial.print("CH5: ");
        Serial.print(grams5, 2);
        Serial.print(" g");
    }
    else
    {
        Serial.print("CH5: --");
    }


    Serial.print("    ");


    if (ok7)
    {
        float grams7 =
            (raw7 - zero7) / factor7;

        if (grams7 < 0)
            grams7 = 0;

        Serial.print("CH7: ");
        Serial.print(grams7, 2);
        Serial.println(" g");
    }
    else
    {
        Serial.println("CH7: --");
    }


    delay(50);
}
