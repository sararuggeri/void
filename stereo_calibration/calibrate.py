import cv2, glob, sys
from pathlib import Path
import numpy as np

PATTERN = (9, 6)       # angoli interni
SQUARE = 0.025         # lato quadrato in metri (25,0 mm misurati)
BASE_DIR = Path(__file__).resolve().parent

objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE

files_l = sorted(str(p) for p in (BASE_DIR / "left").glob("*.png"))
files_r = sorted(str(p) for p in (BASE_DIR / "right").glob("*.png"))
if not files_l or not files_r:
    sys.exit("Cartelle left/right vuote")
if len(files_l) != len(files_r):
    sys.exit("Cartelle left/right contengono un numero diverso di immagini")

crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
objpts, ptsl, ptsr, names = [], [], [], []

for fl, fr in zip(files_l, files_r):
    gl = cv2.imread(fl, cv2.IMREAD_GRAYSCALE)
    gr = cv2.imread(fr, cv2.IMREAD_GRAYSCALE)
    okl, cl = cv2.findChessboardCorners(gl, PATTERN)
    okr, cr = cv2.findChessboardCorners(gr, PATTERN)
    if not (okl and okr):
        print(f"scartata (scacchiera non trovata): {fl}")
        continue
    cl = cv2.cornerSubPix(gl, cl, (11, 11), (-1, -1), crit)
    cr = cv2.cornerSubPix(gr, cr, (11, 11), (-1, -1), crit)
    objpts.append(objp); ptsl.append(cl); ptsr.append(cr); names.append(fl)

size = gl.shape[::-1]
print(f"\nCoppie usate: {len(objpts)}  |  dimensione immagine: {size}")
if len(objpts) < 15:
    print("ATTENZIONE: meno di 15 coppie valide; la calibrazione potrebbe essere instabile")

# 1) Intrinseche di ogni camera
rms_l, Kl, Dl, rvl, tvl = cv2.calibrateCamera(objpts, ptsl, size, None, None)
rms_r, Kr, Dr, rvr, tvr = cv2.calibrateCamera(objpts, ptsr, size, None, None)
print(f"\nRMS sinistra: {rms_l:.3f} px   RMS destra: {rms_r:.3f} px")
print(f"fx,fy sinistra: {Kl[0,0]:.1f}, {Kl[1,1]:.1f}   centro: {Kl[0,2]:.1f}, {Kl[1,2]:.1f}")
print(f"fx,fy destra:   {Kr[0,0]:.1f}, {Kr[1,1]:.1f}   centro: {Kr[0,2]:.1f}, {Kr[1,2]:.1f}")

print("\nErrore per immagine: la diagnostica RMS per vista non e stata inclusa "
      "per evitare un calcolo fuorviante")

# 2) Calibrazione stereo con intrinseche fisse
rms, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
    objpts, ptsl, ptsr, Kl, Dl, Kr, Dr, size,
    criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC)
baseline = float(np.linalg.norm(T))
print(f"\nRMS stereo: {rms:.3f} px")
print(f"T (m) = {T.ravel()}   baseline = {baseline*1000:.1f} mm")
print("T[0] negativo = ordine sinistra/destra corretto" if T[0, 0] < 0
      else "ATTENZIONE: T[0] positivo, left/right probabilmente scambiate")

# 3) Rettificazione
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    Kl, Dl, Kr, Dr, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
m1x, m1y = cv2.initUndistortRectifyMap(Kl, Dl, R1, P1, size, cv2.CV_16SC2)
m2x, m2y = cv2.initUndistortRectifyMap(Kr, Dr, R2, P2, size, cv2.CV_16SC2)

np.savez(str(BASE_DIR / "stereo_calib.npz"), Kl=Kl, Dl=Dl, Kr=Kr, Dr=Dr, R=R, T=T,
         R1=R1, R2=R2, P1=P1, P2=P2, Q=Q, size=size)
fs = cv2.FileStorage(str(BASE_DIR / "stereo_calib.yaml"), cv2.FILE_STORAGE_WRITE)
for k, v in dict(Kl=Kl, Dl=Dl, Kr=Kr, Dr=Dr, R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q).items():
    fs.write(k, v)
fs.release()

# 4) Controllo visivo: una coppia rettificata con righe orizzontali
i = int(sys.argv[1]) if len(sys.argv) > 1 else 0
L = cv2.remap(cv2.imread(files_l[i]), m1x, m1y, cv2.INTER_LINEAR)
Rr = cv2.remap(cv2.imread(files_r[i]), m2x, m2y, cv2.INTER_LINEAR)
vis = np.hstack([L, Rr])
for y in range(0, vis.shape[0], 40):
    cv2.line(vis, (0, y), (vis.shape[1], y), (0, 255, 0), 1)
cv2.imwrite(str(BASE_DIR / "rectified_check.png"), vis)
print("\nSalvati: stereo_calib.npz, stereo_calib.yaml, rectified_check.png")
