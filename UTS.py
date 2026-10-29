"""
3D Mapping Drone - CoppeliaSim Edu (ZMQ Remote API)
Full Pipeline: Trajectory + 2D Slice Scan + 3D Point Cloud + Voxel Grid + Height Color Mapping
"""
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ================== PARAMETER KONFIGURASI ==================
ALT             = 1.0         # Ketinggian terbang drone (m)
TILT_DEG        = 50.0        # Sudut sensor menunduk dari horizontal (derajat)
SENSOR_YAW_RATE = 1.2         # Kecepatan putar sensor (rad/detik) menyapu 360 derajat
VOXEL           = 0.08        # Ukuran grid voxel (m)
X_RANGE         = (-1.6, 1.6) # Batas area sapuan waypoint sumbu X
Y_RANGE         = (-1.6, 1.6) # Batas area sapuan waypoint sumbu Y
LANE_STEP       = 0.5         # Jarak antar jalur sapuan zig-zag (m)
WP_TOL          = 0.15        # Toleransi jarak sampai ke waypoint (m)
SPEED           = 0.20        # Kecepatan gerak target/drone (m/s) - pelan & stabil
SCAN_EVERY      = 2           # Ambil data scan setiap N step simulasi
MAX_RANGE       = 4.0         # Batas jarak maksimum pembacaan sensor (m)
# ===========================================================

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(True)

# 1. Ambil Handle Drone & Target sesuai Scene Hierarchy kamu
drone = sim.getObject('/Quadcopter')
target = sim.getObject(':/target')

# Pastikan jika ada 3DLaserScanner di scene, ia menempel ke drone agar ikut terbang
try:
    scanner_model = sim.getObject(':/3DLaserScanner')
    if sim.getObjectParent(scanner_model) != drone:
        sim.setObjectParent(scanner_model, drone, True)
        sim.setObjectPosition(scanner_model, drone, [0.0, 0.0, -0.05])
except Exception:
    pass

# 2. Cari atau Buat 'depthSensor' sebagai child dari /Quadcopter
created_cam = False
try:
    cam = sim.getObject('/Quadcopter/depthSensor')
    print("Menggunakan '/Quadcopter/depthSensor' yang sudah ada di scene.")
except Exception:
    print("Membuat '/Quadcopter/depthSensor' otomatis pada Quadcopter...")
    fov_default = math.radians(60.0)
    cam = sim.createVisionSensor(
        0, [64, 64, 0, 0],
        [0.05, 4.5, fov_default, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    sim.setObjectParent(cam, drone, False)
    sim.setObjectAlias(cam, 'depthSensor')
    created_cam = True

# 3. Ambil Parameter Intrinsik Kamera
near = sim.getObjectFloatParam(cam, sim.visionfloatparam_near_clipping)
far  = sim.getObjectFloatParam(cam, sim.visionfloatparam_far_clipping)
fov  = sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle)
W, H = sim.getVisionSensorRes(cam)
f = (max(W, H) / 2.0) / np.tan(fov / 2.0)

# Pre-komputasi Grid Piksel -> Arah Sinar (Ray)
us, vs = np.meshgrid(np.arange(W), np.arange(H))
X_RAY = -(us - (W - 1) / 2.0) / f
Y_RAY = (vs - (H - 1) / 2.0) / f


def mat12_to_4x4(m):
    """Konversi matriks 3x4 CoppeliaSim menjadi matriks homogen 4x4."""
    M = np.eye(4)
    M[:3, :] = np.array(m).reshape(3, 4)
    return M


def set_sensor_pose(yaw):
    """Mengatur orientasi sensor menghadap arah (yaw, tilt) relatif terhadap badan drone."""
    t = np.deg2rad(TILT_DEG)
    z = np.array([np.cos(t) * np.cos(yaw), np.cos(t) * np.sin(yaw), -np.sin(t)])
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = [x[0], y[0], z[0], 0.0,
         x[1], y[1], z[1], 0.0,
         x[2], y[2], z[2], -0.05]
    sim.setObjectMatrix(cam, drone, m)


def grab_points():
    """Membaca depth buffer dari sensor dan mengubahnya menjadi point cloud di World Frame."""
    res_data = sim.getVisionSensorDepth(cam, 1)
    buf = res_data[0] if isinstance(res_data, (list, tuple)) else res_data
    d = np.frombuffer(buf, dtype=np.float32).reshape(H, W).astype(np.float64)
    
    if d.max() > 1.0001:          # Jika depth sudah dalam satuan meter
        z = d
    else:                         # Jika depth masih ternormalisasi (0..1)
        z = near + d * (far - near)
        
    if not hasattr(grab_points, 'printed'):
        print(f'[Info Sensor] depth min={d.min():.3f} max={d.max():.3f} | near={near:.2f} far={far:.2f} fov={np.rad2deg(fov):.1f} res={W}x{H}')
        grab_points.printed = True
        
    valid = (z < min(MAX_RANGE, far - 0.02)) & (z > near + 0.02)
    P = np.stack([X_RAY * z, Y_RAY * z, z], axis=-1)[valid]
    if len(P) == 0:
        return np.empty((0, 3))
        
    T = mat12_to_4x4(sim.getObjectMatrix(cam, -1))
    Pw = (T[:3, :3] @ P.T).T + T[:3, 3]
    return Pw


def make_waypoints(num_passes=1):  # Ubah angka 2 sesuai berapa kali putaran yang kamu mau
    """Membuat daftar titik waypoint dengan pola sapuan zig-zag berulang."""
    wps = []
    ys = np.arange(Y_RANGE[0], Y_RANGE[1] + 1e-6, LANE_STEP)
    for p in range(num_passes):
        # Putaran genap maju, putaran ganjil mundur (kebalikannya)
        current_ys = ys if p % 2 == 0 else ys[::-1]
        for i, y in enumerate(current_ys):
            xs = X_RANGE if i % 2 == 0 else X_RANGE[::-1]
            wps.append((xs[0], y, ALT))
            wps.append((xs[1], y, ALT))
    return wps


def voxel_downsample(P, size):
    """Mengurangi jumlah titik menjadi pusat grid voxel berukuran 'size'."""
    if len(P) == 0:
        return np.empty((0, 3))
    idx = np.floor(P / size).astype(np.int64)
    _, uniq = np.unique(idx, axis=0, return_index=True)
    centers = (idx[uniq] + 0.5) * size
    return centers


# ================== MAIN LOOP SIMULASI ==================
cloud_chunks, path = [], []
waypoints = make_waypoints()
wp_i = 0

print("Memulai simulasi pemetaan 3D (Synchronous Stepping Mode)...")
sim.startSimulation()
dt = sim.getSimulationTimeStep()
step = 0
t0 = time.time()
tgt_pos = np.array(sim.getObjectPosition(target, -1))

try:
    while wp_i < len(waypoints):
        wp = waypoints[wp_i]
        
        # Geser target secara bertahap menuju waypoint dengan kecepatan SPEED
        diff = np.array(wp) - tgt_pos
        dist = np.linalg.norm(diff)
        if dist > 1e-6:
            tgt_pos = tgt_pos + diff / dist * min(SPEED * dt, dist)
        sim.setObjectPosition(target, -1, tgt_pos.tolist())

        # Putar orientasi yaw sensor secara kontinu untuk menyapu seluruh ruangan
        yaw = (sim.getSimulationTime() * SENSOR_YAW_RATE) % (2 * np.pi)
        set_sensor_pose(yaw)

        # Catat posisi drone untuk jalur lintasan (Grade C)
        pos = sim.getObjectPosition(drone, -1)
        path.append(pos)

        # Ambil data point cloud setiap SCAN_EVERY step (Grade BC & B)
        if step % SCAN_EVERY == 0:
            pts = grab_points()
            if len(pts) > 0:
                cloud_chunks.append(voxel_downsample(pts, VOXEL / 2))

        # Cek apakah drone sudah mencapai waypoint saat ini
        if np.linalg.norm(np.array(pos) - np.array(wp)) < WP_TOL:
            wp_i += 1
            print(f"Waypoint {wp_i}/{len(waypoints)} tercapai...")

        sim.step()
        step += 1
        if time.time() - t0 > 600:  # Batas waktu pengaman 10 menit
            break
except KeyboardInterrupt:
    print("Simulasi dihentikan oleh pengguna.")
finally:
    sim.stopSimulation()
    time.sleep(0.3)
    if created_cam:
        try:
            sim.removeObjects([cam])
        except Exception:
            pass

# ================== POST-PROCESSING (Grade AB & A) ==================
raw = np.vstack(cloud_chunks) if len(cloud_chunks) > 0 else np.empty((0, 3))
vox = voxel_downsample(raw, VOXEL)
path = np.array(path)

print(f'Total Titik Mentah: {len(raw)} -> Setelah Voxel Downsampling: {len(vox)} titik')
if len(vox) > 0:
    print(f'[Statistik Z Peta] min={vox[:,2].min():.2f}m | median={np.median(vox[:,2]):.2f}m | max={vox[:,2].max():.2f}m')

# Simpan data hasil pemetaan ke file .npy
np.save('map_voxel.npy', vox)
np.save('drone_path.npy', path)
np.save('map_raw.npy', raw)

if len(vox) > 0:
    # --- 1. Plot Scan 2D: Irisan Horizontal Tampak Atas (Bukti Tahap BC) ---
    Z_SLICE = (0.15, 0.25)
    sl = vox[(vox[:, 2] > Z_SLICE[0]) & (vox[:, 2] < Z_SLICE[1])]
    
    fig2, ax2 = plt.subplots(num="Scan 2D", figsize=(6, 6))
    ax2.scatter(sl[:, 0], sl[:, 1], s=8, c='k', label='Hasil scan 2D')
    ax2.plot(path[:, 0], path[:, 1], 'b-', lw=1.5, label='Jalur drone')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title(f'Scan 2D (irisan z = {Z_SLICE[0]}-{Z_SLICE[1]} m)')
    fig2.savefig('2d_scan.png', dpi=200)

    # --- 2. Plot 3D Map: Voxel + Color Mapping Ketinggian (Bukti Tahap B, AB, A) ---
    fig = plt.figure(num="3D Map", figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    
    sc = ax.scatter(vox[:, 0], vox[:, 1], vox[:, 2], c=vox[:, 2], cmap='RdYlGn_r',
                    s=6, alpha=0.6, depthshade=False, zorder=1)
    
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', lw=2.5,
            label='Jalur drone', zorder=10)
    ax.scatter(*path[0], color='black', s=60, marker='o', label='Start', zorder=11)
    ax.scatter(*path[-1], color='magenta', s=60, marker='X', label='Finish', zorder=11)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Map')
    
    zmax = max(vox[:, 2].max(), path[:, 2].max())
    ax.set_zlim(0, zmax * 1.05)
    ax.set_box_aspect((np.ptp(vox[:, 0]), np.ptp(vox[:, 1]), zmax * 1.5))
    ax.view_init(elev=35, azim=-60)
    
    fig.colorbar(sc, label='Ketinggian (m)', shrink=0.6)
    ax.legend()
    plt.savefig('3d_map.png', dpi=200)
    plt.show()