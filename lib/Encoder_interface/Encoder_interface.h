#ifnded ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

#include <Arduino.h>

class Encoder_interface
{
    Encoder_interface()
    {
    }

    void config()
    {
        // interrupt [19,18,2,3]
        EICRA = (1 << ISC20) | (1 << ISC30);
        EICRB = (1 << ISC40) | (1 << ISC50);
        EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
        sei();
    }
};

ISR(INT2_vect)
{
    if (digitalRead(ENC_1_PIN_A) == digitalRead(ENC_1_PIN_B))
    {
        count1--;
    }
    else
    {
        count1++;
    }
}

ISR(INT3_vect)
{
    if (digitalRead(ENC_1_PIN_A) == digitalRead(ENC_1_PIN_B))
    {
        count1++;
    }
    else
    {
        count1--;
    }
}

ISR(INT4_vect)
{
    if (digitalRead(ENC_2_PIN_A) == digitalRead(ENC_2_PIN_B))
    {
        count2++;
    }
    else
    {
        count2--;
    }
}

ISR(INT5_vect)
{
    if (digitalRead(ENC_2_PIN_A) == digitalRead(ENC_2_PIN_B))
    {
        count2--;
    }
    else
    {
        count2++;
    }
}

#endif // ENCODER_INTERFACE_H
