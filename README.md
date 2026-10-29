# 3D Mapping dengan Drone (CoppeliaSim Edu)

**[UTS] Sistem Robot Otonom**
Nama: Darvesh Valubia Nabel | NRP: 5022231170

3D mapping sebuah arena menggunakan drone Quadcopter pada CoppeliaSim Edu. Drone dikontrol dari Python lewat ZMQ Remote API, terbang dengan pola sapuan (lawnmower) di atas arena yang berisi lantai, dinding, dan tiga balok. Target grade: **A**.

## Robot dan Sensor
- Robot: drone Quadcopter
- Sensor: vision sensor (depth), perspektif, 128 x 128 piksel, FOV 60°, clipping 0,1 - 5 m
- Posisi dan orientasi sensor: dipasang pada badan drone, menunduk 50° dari bidang horizontal dan berputar terhadap badan drone, sehingga ground, badan objek, dan puncak objek dapat terdeteksi

## Fitur (sesuai grade)
| Grade | Tugas | Status |
|-------|-------|--------|
| C  | Mengontrol drone dan menampilkan jalur yang dilewati drone | Selesai |
| BC | Scan 2D (irisan z = 0,15 - 0,25 m) | Selesai |
| B  | Scan 3D (depth image menjadi point cloud dengan model kamera pinhole) | Selesai |
| AB | Mengurangi titik menjadi voxel (0,08 m) dengan post process | Selesai |
| A  | Color mapping ketinggian dengan post process | Selesai |

## Hasil
**Scan 2D**

![Scan 2D](2D%20Mapping%20Image.png)

**Peta 3D (voxel + color mapping ketinggian)**

![Peta 3D](3D%20Mapping%20Image.png)

## Video Demo
- Video demo proses mapping: https://drive.google.com/file/d/1a3xOCOI5CSDMy4KO_ycoCw59zIFf9H86/view?usp=sharing
- Video demo hasil 3D mapping: https://drive.google.com/file/d/16X94KCCnr7hZnKUwwEh8VVFYjmO89WE0/view?usp=sharing

## Isi Repository
- `UTS.py` : kode Python (kontrol drone, scan, voxel, color mapping)
- `3DMappingDrone.pdf` : laporan
- `2D Mapping Image.png`, `3D Mapping Image.png` : hasil scan
- file scene CoppeliaSim (`.ttt`)

## Cara Menjalankan
1. Buka file scene `.ttt` di CoppeliaSim Edu.
2. Jalankan simulasi.
3. Jalankan `python UTS.py` (membutuhkan paket `coppeliasim-zmqremoteapi-client`, `numpy`, dan `matplotlib`).
