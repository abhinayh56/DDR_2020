#include "Encoder_interface.h"

Encoder_interface::Encoder_interface()
{
}

void Encoder_interface::config()
{
    // interrupt [19,18,2,3]
    EICRA = (1 << ISC20) | (1 << ISC30);
    EICRB = (1 << ISC40) | (1 << ISC50);
    EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
    sei();
}

void Encoder_interface::encoder_1_int2()
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

void Encoder_interface::encoder_1_int3()
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

void Encoder_interface::encoder_2_int4()
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

void Encoder_interface::encoder_2_int5()
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

ISR(INT2_vect)
{
    Encoder_interface::encoder_1_int2();
}

ISR(INT3_vect)
{
    Encoder_interface::encoder_1_int3();
}

ISR(INT4_vect)
{
    Encoder_interface::encoder_2_int4();
}

ISR(INT5_vect)
{
    Encoder_interface::encoder_2_int5();
}