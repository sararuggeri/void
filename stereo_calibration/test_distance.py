import cv2, numpy as np, sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
c = np.load(str(BASE_DIR / "stereo_calib.npz"))
Kl, Dl, Kr, Dr = c["Kl"], c["Dl"], c["Kr"], c["Dr"]
R1, R2, P1, P2, T = c["R1"], c["R2"], c["P1"], c["P2"], c["T"]
size = tuple(int(v) for v in c["size"])
m1 = cv2.initUndistortRectifyMap(Kl, Dl, R1, P1, size, cv2.CV_16SC2)
m2 = cv2.initUndistortRectifyMap(Kr, Dr, R2, P2, size, cv2.CV_16SC2)
f, B = P1[0, 0], float(np.linalg.norm(T))
print(f"f = {f:.1f} px, baseline = {B*1000:.1f} mm")

cap = cv2.VideoCapture(int(sys.argv[1]) if len(sys.argv) > 1 else 0, cv2.CAP_AVFOUNDATION)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
print("[spazio] congela e seleziona l'oggetto   [q] esci")

while True:
    ok, fr = cap.read()
    if not ok:
        break
    L = cv2.remap(fr[:, :1280], *m1, cv2.INTER_LINEAR)
    cv2.imshow("sinistra rettificata", L)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    if k == ord(" "):
        R = cv2.remap(fr[:, 1280:], *m2, cv2.INTER_LINEAR)
        x, y, w, h = cv2.selectROI("sinistra rettificata", L, False)
        if w == 0 or h == 0:
            continue
        tpl = cv2.cvtColor(L[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
        pad = 4
        y0, y1 = max(0, y - pad), min(size[1], y + h + pad)
        x0, x1 = max(0, x - 300), min(size[0], x + w + 20)
        strip = cv2.cvtColor(R[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(strip, tpl, cv2.TM_CCOEFF_NORMED)
        _, score, _, (mx, my) = cv2.minMaxLoc(res)
        sub = 0.0
        if 0 < mx < res.shape[1] - 1:
            a, b, cc = res[my, mx - 1], res[my, mx], res[my, mx + 1]
            den = a - 2 * b + cc
            if den != 0:
                sub = 0.5 * (a - cc) / den
        disp = x - (x0 + mx + sub)
        dy = (y0 + my) - y
        msg = f"match {score:.2f} | disparità {disp:.2f} px | dy {dy} px"
        msg += f" | Z = {f*B/disp:.2f} m" if disp > 0.5 else " | disparità non valida"
        if score < 0.6:
            msg += "  (match debole: zona poco texturizzata?)"
        print(msg)

cap.release(); cv2.destroyAllWindows()