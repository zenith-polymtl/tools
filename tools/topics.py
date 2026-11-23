"""Table des noms de topics des repos de mission Zenith."""

# Un nom de topic s'écrit ici et nulle part ailleurs.
# Le deuxième segment décide seul de ce qui traverse la radio.
# On ajoute une ligne par PR dans ce repo.

EXTERNAL = '/aeac/external'   # traverse la radio (Zenoh)
INTERNAL = '/aeac/internal'   # reste sur la machine
GCS = '/aeac/gcs'             # reste au sol : nodes du portable et interface de la GCS

# --- Externes : vus par le sol ---
GCS_HEARTBEAT = f'{EXTERNAL}/gcs/heartbeat'
DRONE_HEALTH = f'{EXTERNAL}/drone/health'
DEMO_STATE = f'{EXTERNAL}/demo/state'
MISSION_GO = f'{EXTERNAL}/mission/go'   # le GO du sol (nav_stack init)
PAYLOAD_SET_STATE = f'{EXTERNAL}/payload/set_state'   # service ServoState, un servo à un PWM (remote_control_interface)
TRACKER_STATUS = f'{EXTERNAL}/tracker/status'   # TrackerStatus, état du tracker de la Tâche 2 (tracker)

# --- Internes : drone seulement ---
LINK_OK = f'{INTERNAL}/link_ok'
TAKEOFF_COMPLETED = f'{INTERNAL}/mission/takeoff_completed'   # Bool, fin du décollage (nav_stack)
CONVERT_GPS_TO_LOCAL = f'{INTERNAL}/convert/gps_to_local'     # service ConvertGpsToLocal (nav_stack)
LAPS_COMPLETED = f'{INTERNAL}/mission/laps_completed'         # Bool latched, fin des tours de la Tâche 1 (laps_nav) -> state_machine_t1
TRAFFIC = f'{INTERNAL}/comp/traffic'                          # TrafficArray, le trafic du serveur de compétition à 1 Hz (comp_client) -> guidance
COMP_STATUS = f'{EXTERNAL}/comp/status'                       # CompStatus, état du client du serveur de compétition (hexa_ws comp_client), à chaque paquet envoyé
COMP_TELEMETRY_ACK = f'{EXTERNAL}/comp/telemetry_ack'         # CompTelemetryAck, accusé du serveur : insideBoundary, tooCloseToTraffic (hexa_ws comp_client)
COMP_INFRACTION = f'{EXTERNAL}/comp/infraction'               # CompInfraction, totaux d'infractions et score du serveur (hexa_ws comp_client)
DRIVE_SUBMIT = f'{EXTERNAL}/drive/submit'                     # DriveSubmit, valider ou remettre un fichier de la Phase 2 sur le Google Drive de l'équipe (drive_uploader)
DRIVE_STATUS = f'{EXTERNAL}/drive/status'                     # DriveStatus, réponse de drive_uploader à chaque étape d'une demande

# --- Navigation de la Tâche 1 (aeac-2027, context_research/architecure/Task-1/guidance.md) ---
NAV_MODE = f'{INTERNAL}/nav/mode'                             # NavMode latched, mode de navigation actif (state_machine_t1) -> nodes de mode, guidance
NAV_COMPLETED = f'{INTERNAL}/mission/nav_completed'           # NavMode latched, fin d'un mode de navigation -> state_machine_t1
RC_NAV_MODE = f'{INTERNAL}/rc/nav_mode'                       # NavMode latched, mode de navigation demandé par la manette (remote_control_interface) -> state_machine_t1
NAV_INTENT = f'{INTERNAL}/nav/intent'                         # NavIntent latched, chemin à suivre, une fois par chemin (laps_nav, search_nav, return_nav) -> guidance
LAPS_REFUSAL = f'{INTERNAL}/nav/laps_refusal'                 # String, raison d'un refus de LAPS (laps_nav) -> ui_feed (phase de MISSION_STATUS, 10 s)
NAV_PILOT_VELOCITY = f'{INTERNAL}/nav/pilot_velocity'         # VelocityIntent à 20 Hz, vitesse demandée par le pilote (pilot_nav) -> guidance
GUIDANCE_STATUS = f'{INTERNAL}/nav/guidance_status'           # GuidanceStatus à 10 Hz (guidance) -> nodes de mode, state_machine_t1
GUIDANCE_SUMMARY = f'{EXTERNAL}/nav/guidance_status'          # GuidanceStatus à 1 Hz (guidance) -> UI de la GCS
SEARCH_COMPLETED = f'{INTERNAL}/mission/search_completed'     # Bool latched, fin du survey et des repassages (search_nav) -> state_machine_t1

# --- Interface de la GCS (aeac-2027 workspaces/gcs_ws/gcs_ui, simulés dans src/ros/mock.ts) ---
DRONE_STATUS = f'{EXTERNAL}/drone/status'                     # DroneStatus à 1 Hz, état du drone pour la barre d'état (hexa_ws ui_feed) -> UI
DRONE_POSITION = f'{EXTERNAL}/drone/position'                 # DronePosition à 5 Hz, position du drone pour la carte (hexa_ws ui_feed) -> UI
MISSION_STATUS = f'{EXTERNAL}/mission/status'                 # MissionStatus à 1 Hz, avancement de la mission (hexa_ws ui_feed) -> UI
MISSION_GEOMETRY = f'{EXTERNAL}/mission/geometry'             # MissionGeometry latched, zones et tours du fichier du site (hexa_ws ui_feed) -> UI
TRAFFIC_SUMMARY = f'{EXTERNAL}/comp/traffic'                  # TrafficSummary à 1 Hz, TRAFFIC traduit pour la carte (hexa_ws ui_feed) -> UI
TASK2_STATUS = f'{EXTERNAL}/task2/status'                     # état des échantillons de la Tâche 2, 1 Hz -> UI
IMAGE_REQUEST = f'{EXTERNAL}/camera/image_request'            # service, une image HD de la caméra du drone -> UI
GCS_CLUSTERS = f'{GCS}/clusters'                              # cerfs regroupés (clustering, portable) -> UI
GCS_REPORT_STATUS = f'{GCS}/report/status'                    # état du rapport de la Tâche 1 (report_builder) -> UI
GCS_TRACK = f'{GCS}/track'                                    # trajet du tracker de la Tâche 2 -> UI
GCS_REVIEW_DEER = f'{GCS}/deer/review'                        # service, l'opérateur valide ou rejette un cerf
GCS_ADD_ANOMALY = f'{GCS}/report/add_anomaly'                 # service, une anomalie ajoutée au rapport
GCS_SUBMIT_REPORT = f'{GCS}/report/submit'                    # service, envoi du rapport de la Tâche 1
GCS_SUBMIT_TRACK = f'{GCS}/track/submit'                      # service, envoi du CSV du suivi de la Tâche 2

# --- Gimbal D-80AI (mount ArduPilot, MNT1_TYPE=9 + lua/mount-xf-d80-driver.lua ; hexa_ws gimbal_driver) ---
GIMBAL_RC_IN = f'{INTERNAL}/gimbal/rc_in'                     # RCIn, canaux de la manette du caméraman (rc_receiver)
GIMBAL_CMD = f'{INTERNAL}/gimbal/cmd'                         # GimbalCommand, vitesses + action (gimbal_rc_interface) -> gimbal_mavros
GIMBAL_GCS_TARGET = f'{EXTERNAL}/gimbal/gcs_target'           # GimbalTarget à 10 Hz du clavier du portable (gcs_ws gimbal_keyboard) -> gimbal_target : vitesses voulues (rate_mode), ou cible absolue (c), ou neutre
GIMBAL_GCS_CAMERA = f'{EXTERNAL}/gimbal/gcs_camera'           # UInt32MultiArray à 10 Hz du clavier du portable (gimbal_keyboard) -> gimbal_target : compteurs [zoom avant, zoom arrière, mise au point], un appui = +1
GIMBAL_STATE = f'{EXTERNAL}/gimbal/state'                     # GimbalState à 2 Hz, angles mesurés par la GCU (attitude_status de mavros) et commande perdue (gimbal_target) -> sol
