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

# --- Internes : drone seulement ---
LINK_OK = f'{INTERNAL}/link_ok'
