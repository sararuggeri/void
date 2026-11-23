import cv2, os, sys
from pathlib import Path

CAM_INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 0
W, H = 2560, 720          # larghezza totale del frame affiancato, prova anche 3840x1080
PATTERN = (9, 6)          # angoli interni della scacchiera
BASE_DIR = Path(__file__).resolve().parent

LEFT_DIR = BASE_DIR / "left"
RIGHT_DIR = BASE_DIR / "right"
LEFT_DIR.mkdir(exist_ok=True)
RIGHT_DIR.mkdir(exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_AVFOUNDATION)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

ok, frame = cap.read()
if not ok:
    sys.exit("Impossibile leggere dalla camera. Prova un altro indice: python capture_pairs.py 1")
h, w = frame.shape[:2]
print(f"Risoluzione ricevuta: {w}x{h}  (ogni occhio: {w//2}x{h})")

n = max(
    [int(p.stem) for p in LEFT_DIR.glob("*.png") if p.stem.isdigit()]
    + [int(p.stem) for p in RIGHT_DIR.glob("*.png") if p.stem.isdigit()]
    + [-1]
) + 1
print("Tasti: [s] salva coppia (solo se scacchiera rilevata in entrambe)  [q] esci")

while True:
    ok, frame = cap.read()
    if not ok:
        break
    w = frame.shape[1]
    L, R = frame[:, : w // 2], frame[:, w // 2 :]
    gl = cv2.cvtColor(L, cv2.COLOR_BGR2GRAY)
    gr = cv2.cvtColor(R, cv2.COLOR_BGR2GRAY)
    okl, cl = cv2.findChessboardCorners(gl, PATTERN, cv2.CALIB_CB_FAST_CHECK)
    okr, cr = cv2.findChessboardCorners(gr, PATTERN, cv2.CALIB_CB_FAST_CHECK)

    vis = frame.copy()
    if okl:
        cv2.drawChessboardCorners(vis[:, : w // 2], PATTERN, cl, okl)
    if okr:
        cv2.drawChessboardCorners(vis[:, w // 2 :], PATTERN, cr, okr)
    cv2.putText(vis, f"salvate: {n}  L:{okl} R:{okr}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("stereo", cv2.resize(vis, None, fx=0.5, fy=0.5))

    k = cv2.waitKey(1) & 0xFF
    if k == ord("s") and okl and okr:
        cv2.imwrite(str(LEFT_DIR / f"{n:03d}.png"), L)
        cv2.imwrite(str(RIGHT_DIR / f"{n:03d}.png"), R)
        n += 1
    elif k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
