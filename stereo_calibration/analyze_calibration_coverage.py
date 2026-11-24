"""
Analizza la distribuzione dei punti rilevati nelle immagini
sinistra e destra usate dalla calibrazione stereo.

Requisiti:
    pip install opencv-python numpy
"""

import argparse
import glob
import os
import sys
from pathlib import Path

import cv2
import numpy as np


PATTERN = (9, 6)          # Angoli interni della scacchiera
GRID_SIZE = (5, 5)        # Celle per il grafico di copertura
COVERAGE_THRESHOLD = 0.70 # Percentuale minima consigliata per i quadranti
EDGE_MARGIN = 0.10        # 10% del bordo considerato "zona periferica"
BASE_DIR = Path(__file__).resolve().parent


class CoverageStats:
    """
    Mantiene i punti rilevati e ne ricava metriche leggibili.

    La separazione in una classe evita che la parte grafica venga
    mescolata alla rilevazione delle scacchiere.
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.points = []
        self.board_boxes = []
        self.board_angles = []

        # L'occupazione conta quante immagini diverse, non quanti punti,
        # hanno contribuito a ciascuna cella.
        self.occupied_views = [
            set() for _ in range(GRID_SIZE[0] * GRID_SIZE[1])
        ]

    def add_view(self, view_index, corners):
        points = corners.reshape(-1, 2)
        self.points.append(points)

        xs = points[:, 0]
        ys = points[:, 1]

        left = float(xs.min())
        right = float(xs.max())
        top = float(ys.min())
        bottom = float(ys.max())
        self.board_boxes.append((left, top, right, bottom))

        # L'asse principale della scacchiera fornisce una stima semplice
        # dell'inclinazione. Serve solo come indicazione qualitativa.
        centered = points - points.mean(axis=0)
        covariance = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        major_axis = eigenvectors[:, np.argmax(eigenvalues)]
        angle = np.degrees(np.arctan2(major_axis[1], major_axis[0]))
        angle = abs(angle)
        if angle > 90:
            angle = 180 - angle
        self.board_angles.append(angle)

        for x, y in points:
            column = int((x / self.width) * GRID_SIZE[1])
            row = int((y / self.height) * GRID_SIZE[0])

            column = min(column, GRID_SIZE[1] - 1)
            row = min(row, GRID_SIZE[0] - 1)

            cell = row * GRID_SIZE[1] + column
            self.occupied_views[cell].add(view_index)

    def grid_counts(self):
        counts = np.zeros(GRID_SIZE, dtype=np.float32)
        view_count = len(self.points)

        if view_count == 0:
            return counts

        for cell, occupied in enumerate(self.occupied_views):
            row = cell // GRID_SIZE[1]
            column = cell % GRID_SIZE[1]
            counts[row, column] = len(occupied) / view_count

        return counts

    def quadrant_coverage(self):
        """
        Restituisce la frazione di viste che tocca ciascun quadrante.
        """
        if not self.points:
            return {}

        coverage = {
            "alto-sinistra": set(),
            "alto-destra": set(),
            "basso-sinistra": set(),
            "basso-destra": set(),
        }

        for view_index, points in enumerate(self.points):
            xs = points[:, 0]
            ys = points[:, 1]

            if np.any((xs < self.width / 2) &
                      (ys < self.height / 2)):
                coverage["alto-sinistra"].add(view_index)

            if np.any((xs >= self.width / 2) &
                      (ys < self.height / 2)):
                coverage["alto-destra"].add(view_index)

            if np.any((xs < self.width / 2) &
                      (ys >= self.height / 2)):
                coverage["basso-sinistra"].add(view_index)

            if np.any((xs >= self.width / 2) &
                      (ys >= self.height / 2)):
                coverage["basso-destra"].add(view_index)

        view_count = len(self.points)
        return {
            name: len(view_indices) / view_count
            for name, view_indices in coverage.items()
        }

    def edge_coverage(self):
        """
        Percentuale di viste che contengono angoli vicino ai bordi.
        """
        if not self.points:
            return 0.0

        count = 0
        margin_x = self.width * EDGE_MARGIN
        margin_y = self.height * EDGE_MARGIN

        for points in self.points:
            xs = points[:, 0]
            ys = points[:, 1]

            near_edge = (
                np.any(xs < margin_x) |
                np.any(xs > self.width - margin_x) |
                np.any(ys < margin_y) |
                np.any(ys > self.height - margin_y)
            )

            if near_edge:
                count += 1

        return count / len(self.points)

    def board_size_stats(self):
        """
        Dimensione media della scacchiera rispetto all'immagine.
        """
        fractions = []

        for left, top, right, bottom in self.board_boxes:
            width = (right - left) / self.width
            height = (bottom - top) / self.height
            fractions.append(min(width, height))

        if not fractions:
            return 0.0, 0.0

        return float(np.median(fractions)), float(np.min(fractions))


def detect_corners(image_path):
    image = cv2.imread(image_path)
    if image is None:
        return None, None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Questi parametri devono essere coerenti con calibrate.py.
    found, corners = cv2.findChessboardCorners(gray, PATTERN)

    if not found:
        return image, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6
    )
    corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        criteria
    )

    return image, corners


def analyze_folder(folder, width=None, height=None):
    files = sorted(glob.glob(os.path.join(folder, "*.png")))
    detected_size = (width, height)

    print(f"\nAnalisi cartella: {folder}")
    print(f"Immagini trovate: {len(files)}")

    # Prima rilevazione: usata anche per ricavare la dimensione reale.
    detections = []
    for file_path in files:
        image, corners = detect_corners(file_path)
        detections.append((file_path, image, corners))

        if image is not None and detected_size == (None, None):
            height, width = image.shape[:2]
            detected_size = (width, height)

    width, height = detected_size
    if width is None or height is None:
        print("  Nessuna immagine leggibile")
        return [], None, None

    stats = CoverageStats(width, height)
    rejected = []

    for view_index, (file_path, image, corners) in enumerate(detections):
        if corners is None:
            rejected.append(os.path.basename(file_path))
            continue

        stats.add_view(view_index, corners)

    print(f"Scacchiere rilevate: {len(stats.points)}")
    print(f"Scacchiere scartate: {len(rejected)}")

    if rejected:
        print("  File scartati:")
        for name in rejected:
            print(f"    {name}")

    return detections, stats, (width, height)


def make_heatmap(stats, title):
    grid = stats.grid_counts()

    # Interpolazione solo per rendere la heatmap leggibile.
    # Le percentuali numeriche restano quelle della matrice originale.
    heatmap = cv2.resize(
        grid,
        (600, 600),
        interpolation=cv2.INTER_LINEAR
    )

    heatmap = np.clip(heatmap, 0, 1)
    heatmap_uint8 = np.uint8(heatmap * 255)
    colored = cv2.applyColorMap(heatmap_uint8, cv2.COLORMAP_TURBO)

    cv2.rectangle(
        colored,
        (0, 0),
        (colored.shape[1] - 1, colored.shape[0] - 1),
        (255, 255, 255),
        2
    )

    for index in range(1, GRID_SIZE[0]):
        y = int(index * colored.shape[0] / GRID_SIZE[0])
        cv2.line(
            colored,
            (0, y),
            (colored.shape[1], y),
            (255, 255, 255),
            1
        )

    for index in range(1, GRID_SIZE[1]):
        x = int(index * colored.shape[1] / GRID_SIZE[1])
        cv2.line(
            colored,
            (x, 0),
            (x, colored.shape[0]),
            (255, 255, 255),
            1
        )

    # Percentuali numeriche sovrapposte alle celle.
    for row in range(GRID_SIZE[0]):
        for column in range(GRID_SIZE[1]):
            y = int((row + 0.5) * colored.shape[0] / GRID_SIZE[0])
            x = int((column + 0.5) * colored.shape[1] / GRID_SIZE[1])
            percentage = grid[row, column] * 100

            cv2.putText(
                colored,
                f"{percentage:.0f}%",
                (x - 32, y + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

    cv2.putText(
        colored,
        title,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        colored,
        "Percentuale di viste che copre ogni cella",
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    return colored


def print_diagnostics(name, stats):
    print(f"\nDiagnostica {name}:")
    print(f"  Viste valide: {len(stats.points)}")

    quadrant = stats.quadrant_coverage()
    for region, fraction in quadrant.items():
        print(f"  {region}: {fraction * 100:.1f}%")

    edge_fraction = stats.edge_coverage()
    median_size, minimum_size = stats.board_size_stats()

    print(f"  Copertura vicino ai bordi: {edge_fraction * 100:.1f}%")
    print(f"  Dimensione mediana del board: {median_size * 100:.1f}%")
    print(f"  Dimensione minima del board: {minimum_size * 100:.1f}%")

    if stats.board_angles:
        print(
            "  Angolo scacchiera:"
            f" {np.min(stats.board_angles):.1f} - "
            f"{np.max(stats.board_angles):.1f} gradi"
        )

    red_flags = []

    if len(stats.points) < 15:
        red_flags.append("meno di 15 viste valide")

    for region, fraction in quadrant.items():
        if fraction < COVERAGE_THRESHOLD:
            red_flags.append(
                f"copertura insufficiente in {region}: "
                f"{fraction * 100:.1f}%"
            )

    if edge_fraction < 0.30:
        red_flags.append("pochi angoli vicino ai bordi")

    if minimum_size < 0.15:
        red_flags.append("almeno una vista mostra la scacchiera molto piccola")

    if red_flags:
        print("  RED FLAG:")
        for flag in red_flags:
            print(f"    - {flag}")
    else:
        print("  Nessun red flag evidente dalle statistiche disponibili")


def main():
    parser = argparse.ArgumentParser(
        description="Analizza la distribuzione degli angoli stereo."
    )
    parser.add_argument("--left", default=str(BASE_DIR / "left"))
    parser.add_argument("--right", default=str(BASE_DIR / "right"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    left_files = sorted(glob.glob(os.path.join(args.left, "*.png")))
    right_files = sorted(glob.glob(os.path.join(args.right, "*.png")))

    if not left_files or not right_files:
        print("Le cartelle left/ e right/ non contengono immagini")
        sys.exit(1)

    _, left_stats, left_size = analyze_folder(args.left)
    _, right_stats, right_size = analyze_folder(
        args.right,
        left_size[0] if left_size else None,
        left_size[1] if left_size else None
    )

    if left_stats is None or right_stats is None:
        print("Non è stato possibile analizzare le immagini")
        sys.exit(1)

    print_diagnostics("SINISTRA", left_stats)
    print_diagnostics("DESTRA", right_stats)

    left_heatmap = make_heatmap(left_stats, "CAMERA SINISTRA")
    right_heatmap = make_heatmap(right_stats, "CAMERA DESTRA")

    heatmaps = np.hstack([left_heatmap, right_heatmap])
    cv2.imwrite(str(BASE_DIR / "calibration_coverage.png"), heatmaps)

    print("\nGrafico salvato: calibration_coverage.png")

    if args.show:
        cv2.imshow("Coverage analisi calibrazione", heatmaps)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()