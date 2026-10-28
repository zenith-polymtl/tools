"""Table des noms de topics des repos de mission Zenith."""

# Un nom de topic s'écrit ici et nulle part ailleurs.
# Le deuxième segment décide seul de ce qui traverse la radio.
# On ajoute une ligne par PR dans ce repo.

EXTERNAL = '/aeac/external'   # traverse la radio (Zenoh)
INTERNAL = '/aeac/internal'   # reste sur la machine

# --- Externes : vus par le sol ---
GCS_HEARTBEAT = f'{EXTERNAL}/gcs/heartbeat'
DRONE_HEALTH = f'{EXTERNAL}/drone/health'
DEMO_STATE = f'{EXTERNAL}/demo/state'
MISSION_GO = f'{EXTERNAL}/mission/go'   # le GO du sol (nav_stack init)
PAYLOAD_SET_STATE = f'{EXTERNAL}/payload/set_state'   # service ServoState, un servo à un PWM (remote_control_interface)

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
NAV_INTENT = f'{INTERNAL}/nav/intent'                         # NavIntent latched, chemin à suivre, une fois par chemin (laps_nav, search_nav, return_nav) -> guidance
NAV_PILOT_VELOCITY = f'{INTERNAL}/nav/pilot_velocity'         # VelocityIntent à 20 Hz, vitesse demandée par le pilote (pilot_nav) -> guidance
GUIDANCE_STATUS = f'{INTERNAL}/nav/guidance_status'           # GuidanceStatus à 10 Hz (guidance) -> nodes de mode, state_machine_t1
GUIDANCE_SUMMARY = f'{EXTERNAL}/nav/guidance_status'          # GuidanceStatus à 1 Hz (guidance) -> UI de la GCS
SEARCH_COMPLETED = f'{INTERNAL}/mission/search_completed'     # Bool latched, fin du survey et des repassages (search_nav) -> state_machine_t1
