# Test de banco — OAK + AprilTags + medición de vibración (sin volar)

Verifica que la cámara OAK-D-LITE detecta los dos AprilTags y que el pipeline
de vibración relativa funciona, **sin armar ni volar el drone**.

**No necesitas:** MAVROS, MAVProxy, ni el Pixhawk. El nodo de medición
(`relative_vibration_pose`) solo consume los topics de pose de los tags.

**Sí necesitas:**

- La cámara OAK conectada por USB3 (puerto azul del Pi, sin hub)
- Los dos AprilTags, familia **36h11**: **ID 0** (referencia, fijo)
  e **ID 1** (el "vibrante"). Deben ser **del mismo tamaño** — el PnP usa
  un solo `tag_size_m` para ambos.
- Medir con regla el lado del **cuadro negro** de los tags.
  Los tags actuales miden **165.1 mm** (6.5 in) → `tag_size_m:=0.1651`.
  Si usas otros, mide y ajusta el argumento.

> Nota (2026-07-15): el pipeline de medición se arregló en esta fecha —
> `apriltag_pnp_broadcaster` ahora publica las poses por tag
> (`/apriltag_ref/pose`, `/apriltag_vib/pose`). Los viejos `tag_pose_selector`
> no funcionan en Jazzy (el mensaje de detección ya no trae pose) y se
> quitaron del launch. Si ves el error `Unsupported pose layout ... NO pose`,
> estás corriendo una build vieja: `colcon build --packages-select
> tag_hover_two_tags --symlink-install` y relanza.

En cada terminal, primero:

```bash
cd ~/mars-drone-development-arte
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## Paso 0 — Carpeta del experimento (una vez por corrida)

Cada experimento vive en su propia carpeta dentro de `experiments/`. Antes de
lanzar nada, crea la del día:

```bash
./tools/new_experiment.sh              # → experiments/2026-07-17_123456/
./tools/new_experiment.sh vuelo1       # → experiments/2026-07-17_123456_vuelo1/
```

El script deja el enlace `experiments/current` apuntando a la carpeta nueva —
todos los comandos de abajo escriben a `experiments/current`, así que el CSV,
el video y las gráficas de cada corrida quedan juntos automáticamente.

---

## Terminal 1 — Stack de visión

Cámara + detector AprilTag + PnP broadcaster (TF + poses por tag):

```bash
ros2 launch tag_hover_two_tags hardware_vision_stack_oak.launch.py tag_size_m:=0.1651
```

Espera a ver `Camera ready!` (~5 s). **Si no aparece**, ve a "Si algo falla".

> Nota Pi 5: los parámetros por defecto (720p sin decimación) van sobrados —
> ~22 Hz de detecciones usando ~70 % de un core (medido 2026-07). No hace
> falta `apriltag_params_pi.yaml` salvo que quieras margen extra de CPU.

---

## Terminal 2 — Nodo de medición de vibración

```bash
ros2 run tag_hover_two_tags relative_vibration_pose --ros-args \
    -p csv_dir:=~/mars-drone-development-arte/experiments/current
```

Publica `/relative_vibration_pose` (pose del tag 1 relativa al tag 0) y
guarda un CSV en la carpeta del experimento (sin el parámetro `csv_dir`
caería en `~/.ros/tag_hover_two_tags/`). Debe imprimir
`Logging relative pose to <ruta>` — verifica que la ruta sea la carpeta
del experimento de hoy.

---

## Terminales 3 y 4 — Ver lo que ve la cámara

`tag_overlay` dibuja los recuadros de los tags detectados sobre la imagen:

```bash
# Terminal 3
ros2 run tag_hover_two_tags tag_overlay --ros-args -p image_topic:=/oak/rgb/image_raw

# Terminal 4 — la ventana de video
ros2 run image_view image_view --ros-args -r image:=/image_with_tags
```

Deberías ver el video de la OAK con un recuadro y el ID sobre cada tag
detectado. Si un tag está en cuadro pero sin recuadro, el detector no lo
está reconociendo (luz, distancia, enfoque o familia equivocada).

---

## Terminal 5 — Verificación por topics

Coloca **ambos tags visibles a la vez** frente a la cámara. Con tags de
16.5 cm, ponlos a ~1.5–2 m para que quepan los dos en cuadro. Prueba en orden:

```bash
# 1. ¿Detecta los tags? Debes ver id: 0 e id: 1
ros2 topic echo /detections --no-arr --once

# 2. ¿A qué frecuencia? (esperado en la Pi 5: ~15-20 Hz)
ros2 topic hz /detections

# 3. ¿Fluyen las poses de cada tag? (mismo rate que las detecciones)
ros2 topic hz /apriltag_ref/pose
ros2 topic echo /apriltag_vib/pose --once

# 4. ¿Sale la medición relativa? — ESTE es el resultado final
ros2 topic echo /relative_vibration_pose
```

---

## Terminal 6 — Grabar video de la OAK (opcional pero recomendado)

Grabar el video permite correr después el análisis offline
(`tools/video_vibration_analyzer.py`) y compararlo contra el CSV del pipeline
en vivo — dos caminos independientes al mismo resultado, buen cross-check.

**Solo la primera vez** — guarda la calibración de la OAK en un YAML:

```bash
ros2 topic echo /oak/rgb/camera_info --once
```

Del array `k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]` copia los valores a
`~/oak_rgb.yaml` (con nano, no es un comando):

```yaml
fx: <k[0]>
fy: <k[4]>
cx: <k[2]>
cy: <k[5]>
distortion: [<los 8 valores del array d:>]
```

La calibración no cambia mientras uses la misma resolución — este archivo se
reutiliza para siempre.

**En cada corrida** — mide el frame rate real y graba:

```bash
# 1. Mide el rate (anota el número, Ctrl+C para salir)
ros2 topic hz /oak/rgb/image_raw

# 2. Graba usando ESE número como fps — con decimal, ej. 30.0 (Ctrl+C para terminar)
ros2 run image_view video_recorder --ros-args \
    -r image:=/oak/rgb/image_raw \
    -p filename:=/home/mars/mars-drone-development-arte/experiments/current/experiment.avi \
    -p fps:=30.0 -p codec:=MJPG
```

> **Importante:** el `fps:=` debe coincidir con lo que dijo `topic hz` *mientras
> grabas* (la codificación carga CPU y puede bajar el rate). El analizador de
> video calcula el tiempo como `frame / fps` — un fps equivocado escala todas
> las frecuencias del resultado.

---

## El test de vibración

Con el paso 4 corriendo:

1. **Deja el tag 0 fijo** — pégalo a la pared o a algo que no se mueva.
2. **Mueve el tag 1 con la mano** durante 30–60 s — oscílalo unos centímetros
   de lado a lado.
   → Los valores de `position` en `/relative_vibration_pose` deben oscilar
   siguiendo tu mano. Con `tag_size_m` correcto son **metros reales**: si lo
   mueves 5 cm, debes ver ~0.05 de amplitud.
3. **La prueba clave:** deja los dos tags quietos y mueve *la cámara*
   (simulando el drift del drone).
   → La pose relativa casi **no** debe cambiar. Eso demuestra que la
   cancelación de drift del sistema de dos tags funciona.

---

## Cómo terminar y conseguir los resultados

1. **Ctrl+C en la Terminal 6 primero** (si grabaste video) — cierra el `.avi`.
2. **Ctrl+C en la Terminal 2** — cierra el CSV correctamente.
3. Ctrl+C en el resto de terminales.
4. Verifica que no quedaron procesos huérfanos (causan problemas al relanzar):
   ```bash
   pgrep -af "apriltag|depthai|component_container|relative_vibration"
   # si sale algo: pkill -f apriltag_node  (etc.)
   ```
5. Tus datos — todo junto en la carpeta del experimento:
   ```bash
   ls -la experiments/current/
   ```
   El CSV de la corrida debe pesar **varios KB**. Un archivo de 0 bytes =
   corrida vacía, no llegaron poses (revisa el paso 3 de la Terminal 5).

### Path A — resultados del pipeline en vivo (CSV)

El CSV ya contiene la pose relativa (x, y, z, roll, pitch, yaw en metros y
radianes). Para sacar las gráficas de desplazamiento y frecuencia:

```bash
python3 tools/csv_vibration_analyzer.py experiments/current/relative_vibration_*.csv
```

Imprime el resumen (rate, RMS y Hz dominante por eje) y deja
`*_displacement.png` y `*_frequency.png` junto al CSV.

### Path B — resultados del video grabado

Requiere el video de la Terminal 6 y la calibración `~/oak_rgb.yaml`
(dependencias una sola vez: `pip install -r tools/requirements.txt`):

```bash
python3 tools/video_vibration_analyzer.py experiments/current/experiment.avi \
    --calibration ~/oak_rgb.yaml --tag-size 0.1651 --annotated-video
```

Deja en la misma carpeta del experimento: el CSV por frame,
`experiment_vibration.csv`, `experiment_displacement.png`,
`experiment_frequency.png` y `experiment_annotated.mp4` (el video con las
detecciones dibujadas — **lo primero que hay que revisar** si los resultados
salen raros).

### Comparación

Los dos paths calculan la misma física con el mismo método (detección de
esquinas → PnP → pose relativa `T_vib_ref = inv(T_ref_cam) · T_vib_cam`), así
que la **frecuencia dominante (Hz)** y el **RMS (mm)** deben coincidir entre
las gráficas de A y B. Si difieren mucho, lo más probable es `--tag-size`
equivocado, calibración que no corresponde, o el `fps:=` de la grabación mal
puesto (eso escala las frecuencias del Path B).

---

## Si algo falla

| Síntoma | Causa probable |
|---|---|
| No dice `Camera ready!` | (a) Procesos huérfanos de corridas anteriores acaparando la cámara — revisa `pgrep -af "apriltag\|depthai\|component"` y mátalos. (b) La cámara se cayó del USB — revisa `journalctl -k \| grep -i usb`; si hay `USB disconnect`, revisa cable y fuente (usar la oficial de 27 W). No suele ser temperatura: verifica con `cat /sys/class/thermal/thermal_zone0/temp`. |
| Error `Unsupported pose layout ... NO pose` | Build vieja de antes del arreglo del 2026-07-15. Recompila y relanza. |
| Nada en `/detections` | ¿La cámara publica? `ros2 topic hz /oak/rgb/image_raw`. Revisa luz, distancia, enfoque. |
| `/detections` fluye pero `/apriltag_ref/pose` no | El PnP broadcaster no arrancó o no recibe `camera_info`. Revisa la Terminal 1. |
| Detecta tag 0 pero no tag 1 (o al revés) | Ambos deben estar en cuadro **al mismo tiempo** — la sincronización tolera 50 ms. |
| Los dos tags salen con el mismo ID | Imprimiste dos veces el mismo tag. Necesitas 36h11 **ID 0** e **ID 1** (el patrón de cuadritos codifica el ID). |
| Poses raras o escaladas | `tag_size_m` no coincide con el tamaño real del cuadro negro. |
| CSV en 0 bytes | No llegaron poses al nodo de medición — mismo diagnóstico que las filas anteriores. |
| No abre la ventana de video | `image_view` necesita sesión gráfica (pantalla conectada o VNC, no SSH pelón). |
| El driver no encuentra la cámara | Revisa `lsusb \| grep 03e7` y la regla udev de Movidius. |
