Vai ai contenuti principali
Google Classroom
Classroom
RomeCup 2026 Liceo Landi
Stream
Lavori del corso
Persone
RomeCup 2026 Liceo Landi
In consegna
Nessun lavoro in consegna a breve. Bene!

Post di Gregory Specchi
Gregory Specchi
Data di creazione: 3 dic3 dic
File per frequency detector su Raspberry
frequency_detector.py
Testo

1 commento sul corso

Gregory Specchi • 3 dic
pip install pyaudio numpy

Aggiungi commento sul corso…


Post di Gregory Specchi
Gregory Specchi
Data di creazione: 3 dic3 dic
File per rilevamento frequenza con ESP32 (Arduino)
frequency_detector.ino
File binari

enumsFFT.h
C

defs.h
C

types.h
C

arduinoFFT.h
C

arduinoFFT.cpp
File binari

import pyaudio
import numpy as np
import time

# --- Parametri di Acquisizione ---
CHUNK = 1024           # Dimensione del blocco (buffer) da leggere
FORMAT = pyaudio.paInt16 # Formato audio (16-bit integer)
CHANNELS = 1           # Canali (Mono)
RATE = 44100           # Frequenza di campionamento (Sample Rate) in Hz

# --- Parametri di Rilevamento ---
TARGET_FREQ = 4000     # Frequenza target da rilevare (4000 Hz)
THRESHOLD = 1000000    # Soglia di ampiezza per considerare la frequenza "evidente" (Regola questo valore!)
# Si raccomanda un RATE di almeno 2 * TARGET_FREQ (Teorema di Nyquist). 44100 Hz è comune e sicuro.

p = pyaudio.PyAudio()

# Trova l'indice del microfono USB.
# Potrebbe essere necessario eseguirlo una volta per trovare l'indice corretto.
# Per semplicità, proviamo con l'indice di default (0) o l'indice 1, ma è meglio verificarlo:
# print("Dispositivi disponibili:")
# for i in range(p.get_device_count()):
#     info = p.get_device_info_by_index(i)
#     print(f"Indice {i}: {info['name']}")

# Sostituisci l'indice se il tuo microfono USB non è il dispositivo di default (0)
# Esempio: input_device_index=1 
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print(f"🔴 Ascolto per frequenza a {TARGET_FREQ} Hz...")

try:
    while True:
        # Leggi i dati dal microfono
        data = stream.read(CHUNK, exception_on_overflow=False)
        # Converti i dati in un array numpy
        np_data = np.frombuffer(data, dtype=np.int16)
        
        # --- Trasformata Rapida di Fourier (FFT) ---
        # Calcola l'FFT
        yf = np.fft.fft(np_data)
        # Prendi l'ampiezza (magnitudine) e considera solo la prima metà (parte reale)
        yf_mag = np.abs(yf[:CHUNK//2]) 

        # Calcola le frequenze corrispondenti
        xf = np.fft.fftfreq(CHUNK, 1 / RATE)
        xf = xf[:CHUNK//2]
        
        # --- Rilevamento della Frequenza Target ---
        # Trova l'indice nel vettore delle frequenze che è più vicino alla TARGET_FREQ
        idx = np.argmin(np.abs(xf - TARGET_FREQ))
        
        # Ottieni l'ampiezza a quella frequenza
        magnitude_at_target = yf_mag[idx]

        # --- Valutazione ---
        if magnitude_at_target > THRESHOLD:
            print(f"✅ FREQUENZA A {TARGET_FREQ} Hz EVIDENTE! Ampiezza: {magnitude_at_target:.2f}")
        else:
            print(f"❌ Nessuna frequenza evidente a {TARGET_FREQ} Hz. Ampiezza: {magnitude_at_target:.2f}", end='\r')
            
        time.sleep(0.01) # Breve pausa per non sovraccaricare la CPU

except KeyboardInterrupt:
    print("\nScript interrotto dall'utente.")
    
finally:
    # --- Pulizia ---
    stream.stop_stream()
    stream.close()
    p.terminate()
    print("Flusso audio chiuso.")
frequency_detector.py
Visualizzazione di frequency_detector.py.
