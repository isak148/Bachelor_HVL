# -*- coding: utf-8 -*-
# Husk å installere nødvendige biblioteker:
# pip install mpu6050-raspberrypi numpy scipy

from mpu6050 import mpu6050
from time import sleep
import math
import time

# Importert for FFT-analyse og filtrering
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter

# smbus2 kan være nødvendig internt av mpu6050-biblioteket, men vi bruker det ikke direkte her.

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)

        print("Initialiserer MPU6050...")

        # Juster følsomhetsskala (grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        # Juster akselerometer følsomhet
        self.set_accel_range(self.ACCEL_RANGE_2G)
        # Velg filtrering for MPU. Lavere båndbredde gir mer filtrering, men tregere respons.
        # Prøver med 42Hz i stedet for 188Hz for å redusere støy.
        self.set_filter_range(self.FILTER_BW_42) # Endret fra FILTER_BW_188

        # Kalibrering (øker antall samples for bedre nøyaktighet)
        # VIKTIG: Sørg for at sensoren ligger HELT STILLE og VANNRETT under kalibrering!
        self.gyro_offset = self.calibrate_gyro(samples=500) # Økt antall samples
        self.accel_offset = self.calibrate_accel(samples=500) # Økt antall samples

        # Komplementærfilter-variabler
        self.gyro_angle_x = 0.0 # Akkumulert vinkel fra gyro (roll)
        self.gyro_angle_y = 0.0 # Akkumulert vinkel fra gyro (pitch)
        self.alpha = 0.95 # Komplementær filtervekt (høyere = mer tillit til gyro)
        self.last_time = time.time()

        # Variabler for automatisk justering basert på stabilitet
        self.stable_count_x = 0
        self.stable_count_y = 0
        self.stable_count_z = 0
        # Øker kravet for å anse sensoren som stabil
        self.required_stable_count = 100 # Økt fra 20
        self.threshold = 0.08 # Litt større slingringsmonn for 1G deteksjon

        # Variabler for FFT / Periodisitetsanalyse
        self.sample_rate = 100  # Hz (Antatt samplingsrate for FFT-analyse)
        self.window_size = self.sample_rate * 5  # 5 sekunder med data
        # Buffer kun for rå akselerasjonsdata før filtrering
        self.raw_data_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = None # Sist kjente periodisitetsstatus

        print("MPU6050 Initialisering fullført.")


    def calibrate_accel(self, samples=500): # Økt default samples
        """Kalibrerer akselerometeret ved å finne gjennomsnittlig bias."""
        print(f"Starter akselerometerkalibrering ({samples} samples)...")
        print("Sørg for at sensoren ligger helt stille og vannrett!")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        try:
            for i in range(samples):
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                sleep(0.02) # Liten pause mellom målinger
                if i % 50 == 0:
                    print(f"Kalibrering aksel sample {i}/{samples}")

            offset['x'] /= samples
            offset['y'] /= samples
            # Juster Z for gravitasjon, forutsatt at sensoren er vannrett (Z peker opp eller ned)
            # Hvis Z leser ca +1G, blir offset nær 0 etter dette.
            # Hvis Z leser ca -1G (opp-ned), blir offset nær -2 etter dette (feil?).
            # Det er bedre å bare lagre rå bias og korrigere for G i vinkelberegningen.
            # La oss prøve å *ikke* trekke fra 1.0 her, men heller håndtere det i get_orientation.
            offset['z'] /= samples
            # offset['z'] = offset['z'] / samples - 1.0 # Fjerner denne justeringen herfra

            print("Akselerometerkalibrering fullført.")
            print(f"Accel Offset: {offset}")
            return offset
        except Exception as e:
            print(f"Feil under akselerometerkalibrering: {e}")
            # Returner null-offset ved feil for å unngå krasj
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def calibrate_gyro(self, samples=500): # Økt default samples
        """Kalibrerer gyroskopet ved å finne gjennomsnittlig drift."""
        print(f"Starter gyroskopkalibrering ({samples} samples)...")
        print("Sørg for at sensoren ligger helt stille!")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        try:
            for i in range(samples):
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                sleep(0.02) # Liten pause
                if i % 50 == 0:
                    print(f"Kalibrering gyro sample {i}/{samples}")

            offset['x'] /= samples
            offset['y'] /= samples
            offset['z'] /= samples

            print("Gyroskopkalibrering fullført.")
            print(f"Gyro Offset: {offset}")
            return offset
        except Exception as e:
            print(f"Feil under gyroskopkalibrering: {e}")
            # Returner null-offset ved feil
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}


    def get_orientation(self):
        """Beregner roll og pitch ved hjelp av et komplementært filter."""
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            # Forhindre for stor dt hvis noe henger
            dt = min(dt, 0.1)
            self.last_time = current_time

            # Hent rådata
            accel_data_raw = self.get_accel_data(g=True)
            gyro_data_raw = self.get_gyro_data()

            # Juster for kalibrert bias (offset)
            accel_data = {
                'x': accel_data_raw['x'] - self.accel_offset['x'],
                'y': accel_data_raw['y'] - self.accel_offset['y'],
                # Vi justerer ikke for Z-offset her hvis vi antar at Z-offset var nær 1G.
                # Hvis kalibreringen bare fjerner bias, må vi justere Z også.
                'z': accel_data_raw['z'] - self.accel_offset['z']
            }
            gyro_data = {
                'x': gyro_data_raw['x'] - self.gyro_offset['x'],
                'y': gyro_data_raw['y'] - self.gyro_offset['y'],
                'z': gyro_data_raw['z'] - self.gyro_offset['z']
            }

            # --- Akselerometerbasert vinkelberegning (tyngdekraftreferanse) ---
            # Standard roll beregning (rotasjon rundt X)
            accel_angle_roll = math.atan2(accel_data['y'], accel_data['z']) * (180 / math.pi)

            # Standard pitch beregning (rotasjon rundt Y) - robust versjon
            # Sikrer mot divisjon med null og håndterer store roll-vinkler bedre
            accel_angle_pitch = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * (180 / math.pi)
            # Enklere versjon (kan være ustabil nær +/- 90 grader roll):
            # accel_angle_pitch = math.atan2(-accel_data['x'], accel_data['z']) * (180/math.pi)

            # --- Gyroskopintegrasjon ---
            # Integrer gyro-data for å beregne vinkelendring siden sist
            # Legg til den integrerte endringen til forrige gyro-vinkelestimat
            self.gyro_angle_x += gyro_data['x'] * dt
            self.gyro_angle_y += gyro_data['y'] * dt

            # --- Stabilitetsdeteksjon og Justering ---
            # Sjekk om noen akse er stabil nær +/- 1G
            # Bruker *justert* akselerometerdata for å sjekke nærhet til 1G totalt sett
            # Bruker rå data for å sjekke stabilitet på enkel akse? Nei, bruker justert.
            is_stable_x = abs(abs(accel_data['x']) - 1.0) < self.threshold
            is_stable_y = abs(abs(accel_data['y']) - 1.0) < self.threshold
            is_stable_z = abs(abs(accel_data['z']) - 1.0) < self.threshold

            # Oppdater tellere
            self.stable_count_x = self.stable_count_x + 1 if is_stable_x else 0
            self.stable_count_y = self.stable_count_y + 1 if is_stable_y else 0
            self.stable_count_z = self.stable_count_z + 1 if is_stable_z else 0

            # Bruk akselerometervinkelen som mål for filteret, justert ved stabilitet
            current_accel_target_roll = accel_angle_roll
            current_accel_target_pitch = accel_angle_pitch

            # Vurder om vi skal overstyre målet hvis en akse er stabil over tid
            if self.stable_count_z >= self.required_stable_count:
                # Sensor ligger flatt (eller opp-ned)
                # Anta roll og pitch skal være 0 (eller 180 hvis opp-ned, men filteret bør håndtere det)
                current_accel_target_roll = 0.0
                current_accel_target_pitch = 0.0
                # Valgfritt: "Dra" gyro-estimatet forsiktig mot null for å fjerne akkumulert drift raskere
                # self.gyro_angle_x = self.gyro_angle_x * 0.9 + 0.0 * 0.1
                # self.gyro_angle_y = self.gyro_angle_y * 0.9 + 0.0 * 0.1
                print("STABIL Z: Justerer mot R=0, P=0")
                self.stable_count_z = 0 # Nullstill teller etter bruk

            elif self.stable_count_y >= self.required_stable_count:
                # Sensor ligger på siden (langs X-aksen)
                if accel_data['y'] > 0: # Y peker opp (+1G)
                    current_accel_target_roll = -90.0 # Standard høyrehåndsregel: Negativ roll
                else: # Y peker ned (-1G)
                    current_accel_target_roll = 90.0 # Positiv roll
                current_accel_target_pitch = 0.0 # Pitch bør være nær 0
                # Valgfritt: Dra gyro-estimatet mot målet
                # self.gyro_angle_x = self.gyro_angle_x * 0.9 + current_accel_target_roll * 0.1
                # self.gyro_angle_y = self.gyro_angle_y * 0.9 + current_accel_target_pitch * 0.1
                print(f"STABIL Y: Justerer mot R={current_accel_target_roll}, P=0")
                self.stable_count_y = 0 # Nullstill teller

            elif self.stable_count_x >= self.required_stable_count:
                 # Sensor ligger på "nesen" eller "halen" (langs Y-aksen)
                if accel_data['x'] < 0: # X peker opp (-1G på X)
                    current_accel_target_pitch = 90.0 # Positiv pitch
                else: # X peker ned (+1G på X)
                    current_accel_target_pitch = -90.0 # Negativ pitch
                current_accel_target_roll = 0.0 # Roll bør være nær 0
                 # Valgfritt: Dra gyro-estimatet mot målet
                # self.gyro_angle_x = self.gyro_angle_x * 0.9 + current_accel_target_roll * 0.1
                # self.gyro_angle_y = self.gyro_angle_y * 0.9 + current_accel_target_pitch * 0.1
                print(f"STABIL X: Justerer mot R=0, P={current_accel_target_pitch}")
                self.stable_count_x = 0 # Nullstill teller


            # --- Komplementærfilter ---
            # Kombinerer det raske, men driftende gyro-estimatet med det trege, men stabile (i ro) akselerometer-estimatet.
            # Bruker det justerte akselerometer-målet (current_accel_target_...)
            final_angle_x = self.alpha * self.gyro_angle_x + (1 - self.alpha) * current_accel_target_roll
            final_angle_y = self.alpha * self.gyro_angle_y + (1 - self.alpha) * current_accel_target_pitch

            # Oppdater gyro-estimatene med filterets output for neste iterasjon
            # Dette hjelper å holde gyro-integrasjonen bundet til den filtrerte vinkelen
            self.gyro_angle_x = final_angle_x
            self.gyro_angle_y = final_angle_y

            return {'roll': final_angle_x, 'pitch': final_angle_y}

        except Exception as e:
            print(f"Feil i get_orientation: {e}")
            # Returner forrige kjente (eller null) vinkel ved feil
            return {'roll': self.gyro_angle_x, 'pitch': self.gyro_angle_y}


    # --- Funksjoner for Periodisitetsanalyse (FFT) ---

    def butter_highpass(self, cutoff, fs, order=5):
        """Designer et Butterworth høypassfilter."""
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        """Anvender et høypassfilter på data."""
        b, a = self.butter_highpass(cutoff, fs, order=order)
        # Bruk try/except for å håndtere potensielle feil under filtrering (f.eks. for lite data)
        try:
            y = lfilter(b, a, data)
            return y
        except ValueError as e:
             print(f"Filterfeil (ignorerer): {e}. Data lengde: {len(data)}")
             # Returner original data eller tom array ved feil? La oss returnere tom.
             return np.array([])


    def is_periodic(self, signal, threshold_ratio=0.1, min_significant_freqs=1):
        """
        Vurderer om et signal inneholder et periodisk mønster basert på FFT-analyse.
        Returnerer True hvis periodisk, False ellers.
        """
        N = len(signal)
        if N < 2: # Trenger minst 2 punkter for FFT
            return False

        # Sjekk for konstant signal (ingen variasjon) -> ikke periodisk i denne konteksten
        if np.allclose(signal, signal[0]):
             return False

        fft_values = np.fft.rfft(signal) # Real FFT for reelle signaler
        fft_magnitude = np.abs(fft_values) / N # Normaliser

        # Unngå å basere terskel på DC-komponent (første element) hvis den er stor
        if N > 1:
            max_magnitude = np.max(fft_magnitude[1:]) if len(fft_magnitude) > 1 else fft_magnitude[0]
        else:
             max_magnitude = fft_magnitude[0]

        if max_magnitude == 0: # Unngå divisjon med null hvis signalet er null
            return False

        threshold = threshold_ratio * max_magnitude
        # Teller frekvenser (utenom DC) som er over terskelen
        significant_freqs = np.sum(fft_magnitude[1:] > threshold)

        return significant_freqs >= min_significant_freqs


    def gi_status_aks(self):
        """
        Henter data, utfører periodisitetsanalyse (når nok data er samlet),
        og beregner orientering.
        """
        try:
            # --- Akselerasjon og Periodisitet ---
            accel_data = self.get_accel_data(g=True) # Hent rådata for G-beregning
             # Juster for offset før beregning av total G? Ja.
            ax = accel_data['x'] - self.accel_offset['x']
            ay = accel_data['y'] - self.accel_offset['y']
            az = accel_data['z'] - self.accel_offset['z']
            tot_G = math.sqrt(ax**2 + ay**2 + az**2)

            # Legg til den nyeste *totale G*-målingen i bufferet for FFT
            self.raw_data_buffer.append(tot_G)

            # Når bufferet er fullt, analyser periodisitet
            if len(self.raw_data_buffer) == self.window_size:
                # Filtrer dataene i bufferet
                filtered_data = self.highpass_filter(list(self.raw_data_buffer),
                                                     cutoff=0.5, # Frekvenser under 0.5Hz fjernes
                                                     fs=self.sample_rate,
                                                     order=5)

                # Vurder periodisiteten KUN hvis filtreringen ga gyldig output
                if len(filtered_data) > 0:
                    is_periodic_status = self.is_periodic(filtered_data, threshold_ratio=0.15) # Økt terskel litt
                    self.last_periodicity_status = is_periodic_status
                    print(f"FFT Analyse: Periodisk = {self.last_periodicity_status}")
                else:
                     print("FFT Analyse: Kunne ikke filtrere data, status uendret.")


                # Tøm KUN rådata-bufferet for neste vindu (filtered_data er lokal)
                self.raw_data_buffer.clear()

            # --- Orientering ---
            # Kall get_orientation uavhengig av FFT-analysen
            orientation = self.get_orientation()
            roll = orientation['roll']
            pitch = orientation['pitch']

            # Returner resultatene
            return {
                'total_G': tot_G,
                'roll': roll,
                'pitch': pitch,
                'is_periodic': self.last_periodicity_status # Returnerer sist kjente status
            }
        except Exception as e:
            print(f"Feil i gi_status_aks: {e}")
            # Returner en default-status ved feil
            return {
                'total_G': 0.0,
                'roll': 0.0,
                'pitch': 0.0,
                'is_periodic': None
            }


# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        # Standard I2C-adresse for MPU6050 er 0x68
        mpu = MPU6050_Orientation(0x68, bus=1) # bus=1 er vanlig for Raspberry Pi
    except Exception as main_e:
        print(f"Klarte ikke å initialisere MPU6050. Sjekk tilkobling og I2C-adresse.")
        print(f"Feilmelding: {main_e}")
        exit()

    # Mål-looprate (f.eks. 100 Hz -> 0.01s per loop)
    target_dt = 0.01 # Sekunder

    print("\nStarter hovedløkke...")
    while True:
        try:
            loop_start_time = time.time()

            # Hent all statusinformasjon (inkluderer orienteringsberegning)
            status = mpu.gi_status_aks()

            # Skriv ut status
            print(f"Total G: {status['total_G']:.2f} | "
                  f"Roll: {status['roll']:.2f}° | "
                  f"Pitch: {status['pitch']:.2f}° | "
                  f"Periodisk: {'Ja' if status['is_periodic'] else ('Nei' if status['is_periodic'] is False else 'Venter på data...')}")


            # --- Eksempel på bruk av data ---
            # if abs(status['pitch']) > 45.0:
            #     print("-> Ser ut som svømming?")
            # else:
            #     print("-> Ser ut som flyting?")

            # --- Stabiliser looprate ---
            loop_end_time = time.time()
            elapsed_time = loop_end_time - loop_start_time
            sleep_time = target_dt - elapsed_time
            if sleep_time > 0:
                sleep(sleep_time)
            # else:
            #     # Hvis løkken tok lengre tid enn target_dt, skriv ut en advarsel
            #     print(f"ADVARSEL: Loop tok for lang tid ({elapsed_time:.3f}s)")

        except KeyboardInterrupt:
            print("\nAvslutter program...")
            break
        except Exception as loop_e:
            print(f"Feil i hovedløkke: {loop_e}")
            # Vent litt før neste forsøk ved generell feil
            sleep(1)