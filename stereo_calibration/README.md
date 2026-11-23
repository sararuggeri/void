# Calibrazione stereocamera

Questo modulo contiene gli strumenti off-line per la calibrazione di una coppia
di camere. Non modifica il tracker ROS 2 e non viene usato automaticamente da
`start_boat.sh` o `boat_tracker.py`.

## Flusso

1. Acquisire coppie di scacchiera con `capture_pairs.py`.
2. Analizzare la copertura con `analyze_calibration_coverage.py`.
3. Calcolare le matrici e rettificare con `calibrate.py`.
4. Verificare distanze reali con `test_distance.py`.

## Requisiti

- Python 3.9+
- NumPy
- OpenCV
- Una camera affiancata compatibile con OpenCV sulla piattaforma corrente

Installazione minima:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r stereo_calibration/requirements.txt
```

## Parametri da controllare

Prima di ogni acquisizione verificare:

- Pattern: `9 x 6` angoli interni.
- Lato quadrato: 25 mm nel codice attuale.
- Risoluzione attesa: `1280 x 720` per ogni coppia affiancata.
- Fuoco, esposizione e montaggio camera fissati durante tutta la cattura.
- I file sinistro/destro devono rappresentare la stessa posa e avere lo stesso indice.

Le immagini e i parametri del nostro primo campione sono hardware-specifici.
Per questo motivo `left/`, `right/` e i risultati rigenerati non vengono
tracciati da Git.

## Comandi

Entrare nella cartella del modulo:

```bash
cd stereo_calibration
python3 capture_pairs.py 0
python3 analyze_calibration_coverage.py --show
python3 calibrate.py
python3 calibrate.py 0
python3 test_distance.py 0
```

La cattura salva coppie in `left/*.png` e `right/*.png`. `capture_pairs.py`
usa AVFoundation ed e pensato per macOS; su Linux deve essere adattato il backend
VideoCapture senza modificare il resto della pipeline.

## Interpretazione minima

- RMS mono sotto circa 0.8 px e RMS stereo sotto circa 1.0 px sono una buona
  base iniziale, non una certificazione finale.
- La baseline deve essere plausibile rispetto alla barra stereo montata.
- L'immagine `rectified_check.png` deve allineare le linee epipolari su tutte
  le coppie controllate.
- Prima dell'uso sul veicolo servono distanze di riferimento misurate sul campo
  a 0.5, 1, 2, 3, 5, 7 e 10 m.
- Il matching manuale di `test_distance.py` non deve essere considerato
  automaticamente valido: score, disparita verticale e ripetibilita delle
  misure devono essere verificati.

## File esclusi dal repository

Non committare `.venv/`, catture, output OpenCV, video, report temporanei o
log. Questi artefatti sono generati localmente e possono contenere dati
hardware-specifici o occupare molto spazio.
