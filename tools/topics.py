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
LAPS_COMPLETED = f'{INTERNAL}/mission/laps_completed'         # Bool latched, fin des tours de la Tâche 1 (nav_stack lap)
TRAFFIC = f'{INTERNAL}/comp/traffic'                          # TrafficArray, le trafic du serveur de compétition à 1 Hz (comp_client, ou sim_mocks traffic_mock) -> nav_stack lap
COMP_STATUS = f'{EXTERNAL}/comp/status'                       # CompStatus, état du client du serveur de compétition (hexa_ws comp_client), à chaque paquet envoyé
COMP_TELEMETRY_ACK = f'{EXTERNAL}/comp/telemetry_ack'         # CompTelemetryAck, accusé du serveur : insideBoundary, tooCloseToTraffic (hexa_ws comp_client)
COMP_INFRACTION = f'{EXTERNAL}/comp/infraction'               # CompInfraction, totaux d'infractions et score du serveur (hexa_ws comp_client)
