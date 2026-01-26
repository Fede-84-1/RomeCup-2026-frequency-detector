#include <arduinoFFT.h>
#include <driver/i2s.h>

// --- Configurazione Timer per Stampa Seriale ---
unsigned long last_print_time = 0;
// Stampa lo stato ogni 100 millisecondi per un feedback costante
const unsigned long print_interval = 100; 

// --- Parametri I2S e Pinout (Verifica che questi pin corrispondano al tuo cablaggio) ---
#define I2S_BCK_PIN   21  // Serial Clock (BCLK)
#define I2S_WS_PIN    22  // Word Select (LRCLK)
#define I2S_DATA_PIN  18  // Serial Data (DIN)
#define I2S_PORT      I2S_NUM_0

// Frequenza di campionamento: 16000 Hz è usata per rilevare 4000 Hz
#define SAMPLE_RATE   16000 

// --- Parametri FFT ---
#define SAMPLES       512    // Dimensione Finestra FFT (deve essere potenza di 2)
#define SAMPLING_FREQ 16000.0
#define TARGET_FREQ   4000.0 // Frequenza da rilevare

// da scrivere ancora commento
double target_magnitude

// Calcolo del Bin FFT per 4000 Hz (Bin = Freq_target / (Fs / N))
const int TARGET_BIN = round(TARGET_FREQ / (SAMPLING_FREQ / SAMPLES)); 

// Soglia di Magnitudine: Regola questo valore per la tua taratura
const int THRESHOLD_MAGNITUDE = 40000; 

// --- Buffer e Oggetti FFT ---
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT;

/* Funzione per inizializzare l'I2S
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Master e Ricezione
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Acquisiamo a 32 bit
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false 
  };
*/
  // ⚠️ CORREZIONE ORDINE CAMPI: data_out_num deve precedere data_in_num
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE, // Output (non usato)
    .data_in_num = I2S_DATA_PIN,       // Input (microfono)
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  
  i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
}

// Funzione per acquisire i campioni dall'I2S
void i2s_read_samples(double *samples_buffer, size_t num_samples) {
  int32_t raw_samples[SAMPLES];
  size_t bytes_read;
  
  i2s_read(I2S_PORT, &raw_samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  int32_t samples_read = bytes_read / sizeof(int32_t);

  // Normalizza i campioni grezzi
  for (int i = 0; i < samples_read; i++) {
    // Preleva i 16 bit centrali
    int16_t sample_value = (raw_samples[i] >> 16) & 0xFFFF;
    samples_buffer[i] = (double)sample_value;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Avvio Rilevamento 4000 Hz con FFT ---");
  i2s_install();
  Serial.printf("I2S Installato su GPIO: SCK=%d, WS=%d, SD=%d\n", I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_PIN);
  Serial.printf("FFT Configurato: Fs=%d Hz, N=%d campioni, Bin Target=%d (%.2f Hz)\n", SAMPLE_RATE, SAMPLES, TARGET_BIN, (float)TARGET_BIN * (SAMPLING_FREQ / SAMPLES));
  Serial.printf("Soglia di Rilevamento Magnitudine: %d\n", THRESHOLD_MAGNITUDE);
  Serial.println("------------------------------------");
}

bool sound_reveal (TARGET_BIN, SAMPLES, last_print_time, target_magnitude, print_interval, THRESHOLD_MAGNITUDE){
  if (TARGET_BIN < SAMPLES / 2) {
    target_magnitude = vReal[TARGET_BIN]; 

    // Esegui la stampa solo se è trascorso l'intervallo specificato (100ms)
    if (millis() - last_print_time >= print_interval) {
        
        last_print_time = millis(); // Aggiorna il timer

        if (target_magnitude > THRESHOLD_MAGNITUDE) {
          return true;
        } else {
          return false;
        }
    }
  }
}

void loop() {
  // 1. Acquisizione e Preparazione (Cicla alla massima velocità I2S/FFT)
  i2s_read_samples(vReal, SAMPLES);
  for (int i = 0; i < SAMPLES; i++) {
    vImag[i] = 0.0; 
  }

  // 2. Esecuzione FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES); 

  // 3. Analisi e Stampa (Controllata dal Timer)
  if (TARGET_BIN < SAMPLES / 2) {
    target_magnitude = vReal[TARGET_BIN]; 

    // Esegui la stampa solo se è trascorso l'intervallo specificato (100ms)
    if (millis() - last_print_time >= print_interval) {
        
        last_print_time = millis(); // Aggiorna il timer

        if (target_magnitude > THRESHOLD_MAGNITUDE) {
          // Stampa quando il suono è RILEVATO
          Serial.printf("[!!! RILEVATO !!!] Suono a 4000 Hz. Magnitudine: %.0f\n", target_magnitude);

        } else {
          // Stampa quando il suono è ASSENTE
          Serial.printf("In attesa... (Magnitudine rilevata: %.0f)\n", target_magnitude);
        }
    }
    
    // NOTA: Se devi fare un'azione veloce (es. accendere un LED), 
    // falla qui, fuori dal blocco del timer 'if', per la massima reattività!
    // Esempio: 
    // if (target_magnitude > THRESHOLD_MAGNITUDE) { /* Azione qui */ }
  }
}
