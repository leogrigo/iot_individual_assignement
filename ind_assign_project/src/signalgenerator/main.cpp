#include <Arduino.h>
#include <math.h>

constexpr int DAC_PIN = 25;

// Parametri segnale
constexpr float DAC_OFFSET = 127.0f;        // centro circa a metà scala
constexpr float TWO_PI_F   = 6.28318530718f;

struct SineComponent {
    float frequency_hz;
    float amplitude;
    float phase_rad;
};

// Somma di seni: aggiungi/rimuovi righe per cambiare il segnale generato.
// Mantieni la somma delle ampiezze abbastanza sotto 127 per evitare clipping.
constexpr SineComponent SIGNAL_COMPONENTS[] = {
    {3.0f, 10.0f, 0.0f},
    {7.0f, 25.0f, 0.0f},
    {13.0f, 15.0f, 0.0f},
};

constexpr size_t COMPONENT_COUNT = sizeof(SIGNAL_COMPONENTS) / sizeof(SIGNAL_COMPONENTS[0]);

// Intervallo di aggiornamento DAC
// 1 ms => circa 1000 update/s, sufficiente per partire
constexpr uint32_t UPDATE_PERIOD_US = 1000;

uint64_t t_start_us = 0;
uint64_t last_update_us = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    dacWrite(DAC_PIN, 127);

    t_start_us = micros();
    last_update_us = t_start_us;

    Serial.println("Signal Generator boot");
    Serial.printf("DAC pin: GPIO%d\n", DAC_PIN);
    Serial.printf("Signal components: %u\n", (unsigned int)COMPONENT_COUNT);
    for (size_t i = 0; i < COMPONENT_COUNT; i++) {
        Serial.printf(
            "  %u: freq %.2f Hz, amp %.2f, phase %.2f rad\n",
            (unsigned int)i,
            SIGNAL_COMPONENTS[i].frequency_hz,
            SIGNAL_COMPONENTS[i].amplitude,
            SIGNAL_COMPONENTS[i].phase_rad
        );
    }
    Serial.printf("Update period: %lu us\n", (unsigned long)UPDATE_PERIOD_US);
}

void loop() {
    uint64_t now_us = micros();

    if ((now_us - last_update_us) >= UPDATE_PERIOD_US) {
        last_update_us += UPDATE_PERIOD_US;

        float t = (now_us - t_start_us) / 1000000.0f;

        float sample = DAC_OFFSET;
        for (size_t i = 0; i < COMPONENT_COUNT; i++) {
            const SineComponent& component = SIGNAL_COMPONENTS[i];
            sample += component.amplitude * sinf(
                TWO_PI_F * component.frequency_hz * t + component.phase_rad
            );
        }

        if (sample < 0.0f)   sample = 0.0f;
        if (sample > 255.0f) sample = 255.0f;

        dacWrite(DAC_PIN, (uint8_t)sample);
    }
}
