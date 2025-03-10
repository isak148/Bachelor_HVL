import pandas as pd
import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def analyze_data():
    akselerometer_path = "/home/Bachelor/Bachelor_HVL/akselerometer_data.csv"
    tof_path = "/home/Bachelor/Bachelor_HVL/tof_data.csv"
    trykk_path = "/home/Bachelor/Bachelor_HVL/trykk_data.csv"

    while True:
        try:
            akselerometer_data = pd.read_csv(akselerometer_path)
            tof_data = pd.read_csv(tof_path)
            trykk_data = pd.read_csv(trykk_path)

            # Enkle analyser (eksempler)
            avg_accel = akselerometer_data['total_acceleration'].mean() if 'total_acceleration' in akselerometer_data.columns else "N/A"
            max_dist = tof_data['distance_mm'].max() if 'distance_mm' in tof_data.columns else "N/A"
            avg_pressure = trykk_data['pressure_bar'].mean() if 'pressure_bar' in trykk_data.columns else "N/A"

            logging.info(f"Analyse: Gj. aksel.: {avg_accel}, Maks. avstand: {max_dist}, Gj. trykk: {avg_pressure}")

        except FileNotFoundError:
            logging.warning("Venter på at datafiler skal opprettes.")
        except pd.errors.EmptyDataError:
            logging.warning("Datafilene er tomme.")
        except Exception as e:
            logging.error(f"Feil ved dataanalyse: {e}")

        time.sleep(10)  # Analyser data hver 10. sekund.

if __name__ == "__main__":
    analyze_data()