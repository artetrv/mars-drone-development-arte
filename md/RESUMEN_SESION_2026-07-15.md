# Resumen de sesión — 2026-07-15

Objetivo del día: preparar y correr el **test de banco** (sin volar) del
pipeline OAK + AprilTags + medición de vibración, en la Pi 5.

---

## 1. Verificación del hardware actual

- Confirmado: la companion computer es una **Raspberry Pi 5 Model B (8 GB)**,
  no Pi 4 como decía el README. El UART hacia el Pixhawk es `/dev/ttyAMA0`
  (`/dev/ttyS0` no existe en la Pi 5).
- La cámara es una **OAK-D-LITE** (Luxonis), conectada por USB3 a velocidad
  SUPER. Driver instalado: `depthai_ros_driver` 2.12.2 (v2).
- Verificado en vivo: publica `/oak/rgb/image_raw` y `/oak/rgb/camera_info`
  a 1280×720, frame `oak_rgb_camera_optical_frame` — **coincide con todos los
  defaults de los launch files**, no hay que sobreescribir nada.
- `camera_frame` es consistente entre `hardware_vision_stack_oak.launch.py` y
  `hover_controller_oak.launch.py` ✓

## 2. Benchmark de detección en la Pi 5

Con la OAK a 720p y los parámetros AprilTag por defecto (sin decimación):

- **~22 Hz de detecciones**, `apriltag_node` usa ~70 % de un core y el driver
  ~40 % — sobran ~2.9 cores.
- Conclusión: `apriltag_params_pi.yaml` (quad_decimate=2) queda **opcional**
  en la Pi 5, solo para margen extra de CPU. Se suavizó esa guía en los docs.

## 3. Documentación actualizada

- **README.md**: hardware target Pi 4 → Pi 5 + OAK-D-LITE; sección de hardware
  reescrita (la Pi 5 corre el workspace completo, ya no existe `~/drone-pi`);
  serial corregido a `/dev/ttyAMA0`; arreglados los links rotos a los docs
  (se movieron a `md/` en el commit "added new camera" y el README seguía
  apuntando a las rutas viejas).
- **md/INSTALL.md**: nota de hardware actual (Pi 5 + OAK) al inicio; la
  sección 10 quedó marcada como legacy (Pi 4 + RealSense D455).
- **md/BENCH_TEST_OAK.md**: creado — guía completa del test de banco
  (terminales, verificación, cómo cerrar y obtener resultados, tabla de
  fallas). Actualizado al final del día con todo lo aprendido.

## 4. Los AprilTags

- Problema encontrado: los dos tags impresos eran **el mismo ID 0** — el
  patrón de cuadritos codifica el ID, y el sistema necesita IDs distintos
  para distinguir referencia de vibrante.
- Se necesitan: familia **36h11**, **ID 0** (referencia, fijo) e **ID 1**
  (vibrante). Además deben ser **del mismo tamaño** (el PnP usa un solo
  `tag_size_m` para ambos; tamaños distintos rompen la cancelación de drift).
- Tags definitivos conseguidos: 6.5 in = **165.1 mm** de cuadro negro →
  `tag_size_m:=0.1651`. Verificado con el overlay: detecta ID 0 e ID 1 ✓

## 5. Bug encontrado y arreglado: la medición no grababa nada

**Síntoma:** `tag_pose_selector` spameaba
`Failed to extract pose ... NO pose`; los 6 CSVs del día estaban en 0 bytes;
`/apriltag_ref/pose`, `/apriltag_vib/pose` y `/relative_vibration_pose`
nunca publicaron.

**Causa:** en ROS 2 Jazzy, `apriltag_msgs/AprilTagDetection` ya **no trae
pose** (solo ID, esquinas, homografía). Los `tag_pose_selector` fueron
escritos para un mensaje estilo ROS 1 que sí la traía — nunca pudieron
funcionar en Jazzy. La pose real la calcula `apriltag_pnp_broadcaster`
(solvePnP), pero solo la publicaba como TF.

**Arreglo aplicado (commit pendiente):**

- `apriltag_pnp_broadcaster` ahora publica también `PoseStamped` por tag:
  `/apriltag_ref/pose` (ID 0) y `/apriltag_vib/pose` (ID 1), con parámetros
  `ref_tag_id`/`vib_tag_id`/`ref_pose_topic`/`vib_pose_topic`. Ambas poses
  llevan el timestamp de la misma imagen → sincronización exacta para el
  nodo de vibración. El TF (que usa el controlador de vuelo) no cambió.
- `hardware_vision_stack_oak.launch.py`: se quitaron los dos selectores
  rotos; los IDs se pasan al broadcaster.
- Paquete recompilado (`colcon build --packages-select tag_hover_two_tags
  --symlink-install`).

## 6. Debug del "no dice Camera ready!"

- **No era temperatura**: CPU a 34 °C, sin throttling.
- Causa real: **procesos huérfanos** de dos corridas anteriores (los Ctrl+C
  no mataron todo — quedaban 2 `apriltag_node` y 4 selectores), más una
  **desconexión USB** de la cámara a los ~2 min del launch (visible en
  `journalctl -k`). Se limpiaron los huérfanos.
- Si la desconexión USB se repite: revisar cable USB3 y fuente (usar la
  oficial de 27 W); conectar directo al puerto azul, sin hub.

---

## Pendientes

- [ ] **Repetir el experimento de banco** con el arreglo — ahora sí debe
      llenar el CSV (`~/.ros/tag_hover_two_tags/`). Ver
      `md/BENCH_TEST_OAK.md`.
- [ ] Graficar el CSV (posición relativa vs tiempo + espectro de frecuencia).
- [ ] **Commitear** todos los cambios del día (README, INSTALL, bench doc,
      fix del broadcaster, launch OAK — este último ya traía un refactor
      sin commitear de antes: arg `tag_size_m` + include condicional).
- [ ] Decidir si cambiar el default de `tag_size_m` de 0.0673 a 0.1651 en el
      launch, ahora que los tags reales miden 165.1 mm (evita error de
      escala 2.45× si se olvida el argumento en vuelo).
- [ ] El mismo bug de los selectores existe en `sim_vision_stack.launch.py`
      y `hardware_vision_stack.launch.py` (D455) — aplicar el mismo arreglo
      si se vuelven a usar.
- [ ] Para vuelo real: verificar `SERIALx_BAUD` del Pixhawk vs los 57600 del
      README, y probar el flujo MAVProxy → MAVROS por `/dev/ttyAMA0`
      (`./check_mavros_setup.sh`).
