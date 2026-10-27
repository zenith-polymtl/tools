#!/usr/bin/env python3
"""Hands in the Phase 2 deliverables on the team's Google Drive: Task 1 survey (.txt) and Task 2 track (.csv).

Another node writes the file (report_builder, track_csv_writer), then asks for it on topics.DRIVE_SUBMIT
(DriveSubmit): upload false to check it, upload true once the operator confirms (force true to accept
the warnings). Each step is answered on topics.DRIVE_STATUS (DriveStatus, same request_id). Both topics
are external: the node may run on the ground or on the Jetson. Only the tasks in `tasks` are handled,
so that two instances never hand in the same file.

Checks: errors (the format the judges expect) block, even with force; warnings (content probably
wrong) need force. Task 1, four labelled lines, built with format_task1_report():

    Total deer: 8
    Deer per cluster (10 m radius): 2, 2, 3, 1
    Tag IDs: H7, K4, 3P
    Anomalous deer: One deer is wearing a hat.

Task 2 (CONOPS Appendix D): a header line, then timestamp (ISO 8601), latitude, longitude (decimal
degrees), in chronological order.

Upload: the file is read once, and those bytes are checked and sent. The Drive file of the same name in
folder_id is replaced (Drive keeps the old versions), or created; UPLOADED once the md5 Drive computed
matches. drive_modified_time is Google's clock: the hand-in time the judges see. Google Drive API v3
with urllib, OAuth refresh token of the team account, scope drive. Network and file work run in a
worker thread; a ROS timer publishes what it produces.

Credentials: never in git, never in a log. DRIVE_CLIENT_ID, DRIVE_CLIENT_SECRET, DRIVE_REFRESH_TOKEN
from the machine's .env through compose. Without them, the node only validates.

Parameters: team_name, folder_id, task2_name_template (config/drive_uploader.yaml); tasks (launch file);
boundary.lat, boundary.lon (config/sites/<site>.yaml, optional). Guide: doc/drive_uploader.md.
"""
import csv
import hashlib
import io
import json
import math
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.msg import DriveStatus, DriveSubmit
from tools import topics

# --- Noms de remise ---
TASK1_NAME_TEMPLATE = '{team}_task1_survey.txt'   # imposé par le CONOPS (§5.2.3, point 9)
TASK_LABELS = {DriveSubmit.TASK_1_SURVEY: 'Tâche 1', DriveSubmit.TASK_2_TRACKING: 'Tâche 2'}

# --- Tâche 1 ---
LABEL_TOTAL = 'Total deer'
LABEL_CLUSTERS = 'Deer per cluster (10 m radius)'
LABEL_TAGS = 'Tag IDs'
LABEL_ANOMALY = 'Anomalous deer'
T1_LABELS = (LABEL_TOTAL, LABEL_CLUSTERS, LABEL_TAGS, LABEL_ANOMALY)
NONE_WORDS = {'', 'none', 'n/a', '-'}           # liste vide (groupes ou étiquettes)
TAG_ID = re.compile(r'[A-Z0-9]{2}')             # « 2-character capitalized alphanumeric ID codes »

# --- Tâche 2 ---
CSV_HEADER = ('timestamp', 'latitude', 'longitude')
SPEED_WARN_MPS = 7.5        # l'animal ne dépasse pas 5 m/s (§5.2.4, 6b) ; marge pour le bruit GPS
SPEED_WINDOW_S = 2.0        # vitesse mesurée entre deux points espacés d'au moins ça
TRACK_DURATION_S = 300.0    # la trajectoire notée dure 5 minutes (§5.2.4, 6c)
MAX_GAP_S = 5.0             # trou plus long : l'interpolation linéaire coupe les virages
MAX_ERRORS = 10             # au-delà, la suite du CSV n'est pas vérifiée
# ISO 8601, format étendu (2027-05-16T15:04:05.123Z) ou de base (20270516T150405Z). Le T est obligatoire.
ISO_EXTENDED = re.compile(
    r'(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2})(?::(\d{2})(?:[.,](\d+))?)?(Z|[+-]\d{2}(?::?\d{2})?)?')
ISO_BASIC = re.compile(
    r'(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(?:(\d{2})(?:[.,](\d+))?)?(Z|[+-]\d{2}(?:\d{2})?)?')
EARTH_RADIUS_M = 6371000.0

# --- Google Drive ---
CREDENTIAL_ENVS = ('DRIVE_CLIENT_ID', 'DRIVE_CLIENT_SECRET', 'DRIVE_REFRESH_TOKEN')
TOKEN_URL = 'https://oauth2.googleapis.com/token'
API_URL = 'https://www.googleapis.com/drive/v3/files'
UPLOAD_URL = 'https://www.googleapis.com/upload/drive/v3/files'
FILE_FIELDS = 'id,name,md5Checksum,modifiedTime,webViewLink'
MIME_TYPES = {'.txt': 'text/plain', '.csv': 'text/csv'}
HTTP_TIMEOUT_S = 20.0
TOKEN_MARGIN_S = 60.0       # le jeton d'accès est renouvelé ce temps avant son expiration
EVENT_PERIOD_S = 0.1        # le timer ROS qui publie ce que produit le thread de travail


@dataclass
class Report:
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    summary: str = ''

    @property
    def ok(self):
        return not self.errors


def remote_name(template, team_name):
    """Drive file name from a template ('{team}_task1_survey.txt'). Raises ValueError when the team name is unusable."""
    team = (team_name or '').strip()
    if not team:
        raise ValueError("team_name vide : le remplir dans config/drive_uploader.yaml (le nom d'équipe de l'inscription)")
    if any(c in team for c in '/\\\'"') or any(ord(c) < 32 for c in team):
        raise ValueError(f'team_name invalide ({team!r}) : pas de / \\ \' " ni de caractère de contrôle')
    if '{team}' not in template:
        raise ValueError(f'gabarit de nom sans {{team}} : {template!r}')
    return template.replace('{team}', team)


def decode(data, report):
    """UTF-8 text (a BOM is dropped), or None with an error added."""
    if not data.strip():
        report.errors.append('Fichier vide.')
        return None
    try:
        return data.decode('utf-8-sig')
    except UnicodeDecodeError as e:
        report.errors.append(f"Pas de l'UTF-8 (octet {e.start}) : réécrire le fichier en UTF-8.")
        return None


# ---------------------------------------------------------------- Tâche 1

def format_task1_report(total, clusters, tag_ids, anomaly):
    """The Task 1 report in the format validate_task1() checks; for report_builder."""
    return (f'{LABEL_TOTAL}: {total}\n'
            f'{LABEL_CLUSTERS}: {", ".join(str(n) for n in clusters) or "none"}\n'
            f'{LABEL_TAGS}: {", ".join(tag_ids) or "none"}\n'
            f'{LABEL_ANOMALY}: {anomaly.strip() or "none observed"}\n')


def split_list(value):
    return [] if value.strip().lower() in NONE_WORDS else [v.strip() for v in value.split(',')]


def validate_task1(data):
    report = Report()
    text = decode(data, report)
    if text is None:
        return report

    values = {}
    for number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        label, sep, value = line.partition(':')
        known = next((l for l in T1_LABELS if l.lower() == label.strip().lower()), None)
        if not sep or known is None:
            report.warnings.append(f'Ligne {number} non reconnue, remise telle quelle mais non vérifiée : {line.strip()[:60]!r}')
        elif known in values:
            report.errors.append(f'Ligne {number} : « {known} » apparaît deux fois.')
        else:
            values[known] = value.strip()
    for label in T1_LABELS:
        if label not in values:
            report.errors.append(f'Ligne « {label}: ... » absente.')

    total = None
    if LABEL_TOTAL in values:
        if re.fullmatch(r'\d+', values[LABEL_TOTAL]):
            total = int(values[LABEL_TOTAL])
        else:
            report.errors.append(f'« {LABEL_TOTAL} » doit être un entier positif ou nul : {values[LABEL_TOTAL]!r}.')

    clusters = None
    if LABEL_CLUSTERS in values:
        items = split_list(values[LABEL_CLUSTERS])
        bad = [c for c in items if not re.fullmatch(r'[1-9]\d*', c)]
        if bad:
            report.errors.append(f'« {LABEL_CLUSTERS} » : entiers ≥ 1 séparés par des virgules (ou none), pas {bad}.')
        else:
            clusters = [int(c) for c in items]

    tags = None
    if LABEL_TAGS in values:
        tags = split_list(values[LABEL_TAGS])
        bad = [t for t in tags if not TAG_ID.fullmatch(t)]
        if bad:
            report.errors.append(f'« {LABEL_TAGS} » : 2 caractères A-Z ou 0-9, en majuscules, séparés par des virgules (ou none), pas {bad}.')
            tags = None
        else:
            duplicates = sorted({t for t in tags if tags.count(t) > 1})
            if duplicates:
                report.warnings.append(f'Étiquettes en double : {", ".join(duplicates)}.')

    if LABEL_ANOMALY in values and values[LABEL_ANOMALY].lower() in NONE_WORDS:
        report.warnings.append(f'« {LABEL_ANOMALY} » vide : écrire la description, ou « none observed » si aucun.')
    if total is not None and clusters is not None and sum(clusters) != total:
        report.warnings.append(f'Les groupes font {sum(clusters)} cerfs, le total en annonce {total}.')
    if total is not None and tags is not None and len(set(tags)) > total:
        report.warnings.append(f'{len(set(tags))} étiquettes pour {total} cerfs.')

    if total is not None and clusters is not None and tags is not None:
        report.summary = (f'{total} cerfs, {len(clusters)} groupes ({", ".join(map(str, clusters)) or "aucun"}), '
                          f'étiquettes : {", ".join(tags) or "aucune"}')
    return report


# ---------------------------------------------------------------- Tâche 2

def parse_iso8601(text):
    """(aware datetime, has_timezone) or None. A timestamp without a time zone is read as UTC."""
    m = ISO_EXTENDED.fullmatch(text) or ISO_BASIC.fullmatch(text)
    if not m:
        return None
    year, month, day, hour, minute, second, fraction, zone = m.groups()
    try:
        t = datetime(int(year), int(month), int(day), int(hour), int(minute), int(second or 0),
                     tzinfo=timezone.utc)
    except ValueError:
        return None
    if fraction:
        t += timedelta(seconds=float('0.' + fraction))
    if zone and zone != 'Z':
        digits = zone[1:].replace(':', '')
        offset = timedelta(hours=int(digits[:2]), minutes=int(digits[2:4] or 0))
        if offset >= timedelta(hours=24):
            return None
        t -= offset if zone[0] == '+' else -offset
    return t, zone is not None


def distance_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation: a few km at most, far from the poles."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return EARTH_RADIUS_M * math.hypot(x, y)


def inside(lat, lon, polygon):
    """Ray casting on (lat, lon) vertices; the polygon may be non-convex."""
    result = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        (lat_i, lon_i), (lat_j, lon_j) = polygon[i], polygon[j]
        if (lat_i > lat) != (lat_j > lat) and lon < (lon_j - lon_i) * (lat - lat_i) / (lat_j - lat_i) + lon_i:
            result = not result
        j = i
    return result


def validate_task2(data, boundary=None):
    """boundary: [(lat, lon), ...] of the flight boundary, or None to skip that check."""
    report = Report()
    text = decode(data, report)
    if text is None:
        return report
    try:
        rows = [(n, row) for n, row in enumerate(csv.reader(io.StringIO(text)), start=1) if any(c.strip() for c in row)]
    except csv.Error as e:
        report.errors.append(f'CSV illisible : {e}.')
        return report
    if not rows:
        report.errors.append('Aucune ligne.')
        return report

    header = rows[0][1]
    if header and parse_iso8601(header[0].strip()):
        report.errors.append("Ligne 1 : la première ligne doit être l'en-tête des colonnes (timestamp,latitude,longitude), pas un point.")
    else:
        rows = rows[1:]
        if len(header) != 3:
            report.errors.append(f'En-tête : 3 colonnes attendues (timestamp,latitude,longitude), {len(header)} trouvées.')
    if not rows:
        report.errors.append("Aucun point après l'en-tête.")
        return report

    points = []          # (n, t, lat, lon)
    zones = set()
    for n, row in rows:
        if len(report.errors) >= MAX_ERRORS:
            report.errors.append(f'Plus de {MAX_ERRORS} erreurs : la suite du fichier n\'est pas vérifiée.')
            return report
        if len(row) != 3:
            report.errors.append(f'Ligne {n} : 3 colonnes attendues, {len(row)} trouvées ({",".join(row)[:60]!r}).')
            continue
        stamp, lat_text, lon_text = (cell.strip() for cell in row)
        parsed = parse_iso8601(stamp)
        if parsed is None:
            report.errors.append(f'Ligne {n} : horodatage non ISO 8601 : {stamp!r} (ex. 2027-05-16T15:04:05.123Z).')
            continue
        try:
            lat, lon = float(lat_text), float(lon_text)
        except ValueError:
            report.errors.append(f'Ligne {n} : latitude et longitude en degrés décimaux attendues : {lat_text!r}, {lon_text!r}.')
            continue
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            report.errors.append(f'Ligne {n} : coordonnées hors limites ({lat}, {lon}) ; latitude et longitude inversées ?')
            continue
        zones.add(parsed[1])
        points.append((n, parsed[0], lat, lon))

    if len(zones) > 1:
        report.errors.append('Fuseau horaire présent sur certaines lignes seulement : les horodatages sont ambigus.')
    elif zones == {False}:
        report.warnings.append('Horodatages sans fuseau horaire : les juges pourraient les lire en heure locale. Préférer Z (UTC).')
    for (n0, t0, _, _), (n1, t1, _, _) in zip(points, points[1:]):
        if t1 <= t0:
            report.errors.append(f'Ligne {n1} : horodatage {"égal" if t1 == t0 else "antérieur"} à celui de la ligne {n0} (ordre chronologique strict exigé).')
            break
    if report.errors or not points:
        return report

    first, last = points[0][1], points[-1][1]
    duration = (last - first).total_seconds()
    if duration < TRACK_DURATION_S:
        report.warnings.append(f'Le suivi couvre {duration:.0f} s : la trajectoire notée dure {TRACK_DURATION_S:.0f} s.')
    long_gaps = [(a[0], (b[1] - a[1]).total_seconds()) for a, b in zip(points, points[1:])
                 if (b[1] - a[1]).total_seconds() > MAX_GAP_S]
    if long_gaps:
        n, dt = max(long_gaps, key=lambda g: g[1])
        report.warnings.append(f'{len(long_gaps)} trou(s) de plus de {MAX_GAP_S:.0f} s, le plus long {dt:.0f} s après la ligne {n}.')

    fast, j = [], 0
    for i in range(len(points)):
        j = max(j, i + 1)
        while j < len(points) and (points[j][1] - points[i][1]).total_seconds() < SPEED_WINDOW_S:
            j += 1
        if j == len(points):
            break
        speed = distance_m(*points[i][2:], *points[j][2:]) / (points[j][1] - points[i][1]).total_seconds()
        if speed > SPEED_WARN_MPS:
            fast.append((points[i][0], speed))
    if fast:
        n, speed = max(fast, key=lambda f: f[1])
        report.warnings.append(f'{len(fast)} passage(s) à plus de {SPEED_WARN_MPS} m/s, jusqu\'à {speed:.1f} m/s à partir de la ligne {n}.')

    if boundary:
        outside = [n for n, _, lat, lon in points if not inside(lat, lon, boundary)]
        if outside:
            report.warnings.append(f'{len(outside)} point(s) hors de la limite de vol, dont la ligne {outside[0]}.')

    rate = (len(points) - 1) / duration if duration > 0 else 0.0
    report.summary = (f'{len(points)} points, {first:%H:%M:%S} à {last:%H:%M:%S} UTC, '
                      f'{duration:.0f} s, {rate:.1f} Hz en moyenne')
    return report


def validate(task, data, boundary=None):
    return validate_task1(data) if task == DriveSubmit.TASK_1_SURVEY else validate_task2(data, boundary)


# ---------------------------------------------------------------- Google Drive

class DriveError(Exception):
    pass


def http(method, url, body=None, headers=None):
    """JSON answer of an HTTPS request. Raises DriveError; no retry, the operator asks again."""
    try:
        req = urllib.request.Request(url, data=body, method=method, headers=headers or {})
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as answer:
            payload = answer.read()
            return json.loads(payload) if payload else {}
    except urllib.error.HTTPError as e:
        detail = e.read().decode('utf-8', 'replace')
        if 'invalid_grant' in detail:
            raise DriveError("refresh token refusé (révoqué, ou expiré après 7 jours si l'app OAuth est en mode Test) : "
                             'en refaire un (doc/drive_uploader.md)') from None
        raise DriveError(f'HTTP {e.code} : {detail.strip()[:300]}') from None
    except (urllib.error.URLError, OSError) as e:
        raise DriveError(f'réseau : {getattr(e, "reason", e)}') from None


class Drive:
    def __init__(self, client_id, client_secret, refresh_token):
        self.client_id, self.client_secret, self.refresh_token = client_id, client_secret, refresh_token
        self.access_token, self.expiry = None, 0.0

    def auth(self):
        if not self.access_token or time.monotonic() > self.expiry - TOKEN_MARGIN_S:
            answer = http('POST', TOKEN_URL, urllib.parse.urlencode({
                'client_id': self.client_id, 'client_secret': self.client_secret,
                'refresh_token': self.refresh_token, 'grant_type': 'refresh_token'}).encode(),
                {'Content-Type': 'application/x-www-form-urlencoded'})
            self.access_token = answer['access_token']
            self.expiry = time.monotonic() + float(answer.get('expires_in', 3600))
        return {'Authorization': f'Bearer {self.access_token}'}

    def upload(self, folder_id, name, data):
        """Create or replace `name` in the folder; checks the md5. Returns (metadata, replaced)."""
        query = urllib.parse.urlencode({
            'q': f"name = '{name}' and '{folder_id}' in parents and trashed = false",
            'fields': 'files(id)', 'orderBy': 'modifiedTime desc',
            'supportsAllDrives': 'true', 'includeItemsFromAllDrives': 'true'})
        existing = http('GET', f'{API_URL}?{query}', headers=self.auth()).get('files', [])
        mime = MIME_TYPES.get(os.path.splitext(name)[1].lower(), 'application/octet-stream')
        metadata = {'name': name, 'mimeType': mime}
        if not existing:
            metadata['parents'] = [folder_id]
        boundary = uuid.uuid4().hex
        body = b''.join([
            f'--{boundary}\r\nContent-Type: application/json; charset=UTF-8\r\n\r\n'.encode(),
            json.dumps(metadata).encode(),
            f'\r\n--{boundary}\r\nContent-Type: {mime}\r\n\r\n'.encode(),
            data,
            f'\r\n--{boundary}--\r\n'.encode()])
        query = urllib.parse.urlencode({'uploadType': 'multipart', 'supportsAllDrives': 'true', 'fields': FILE_FIELDS})
        headers = dict(self.auth(), **{'Content-Type': f'multipart/related; boundary={boundary}'})
        if existing:
            meta = http('PATCH', f'{UPLOAD_URL}/{existing[0]["id"]}?{query}', body, headers)
        else:
            meta = http('POST', f'{UPLOAD_URL}?{query}', body, headers)
        if meta.get('md5Checksum') != hashlib.md5(data).hexdigest():
            raise DriveError('md5 différent après upload : le fichier sur le Drive n\'est pas celui validé, remettre à nouveau')
        return meta, bool(existing)


# ---------------------------------------------------------------- node

class DriveUploader(Node):
    def __init__(self):
        super().__init__('drive_uploader')
        self.declare_parameter('team_name', '')
        self.declare_parameter('folder_id', '')
        self.declare_parameter('task2_name_template', '{team}_task2_tracking.csv')
        self.declare_parameter('tasks', [DriveSubmit.TASK_1_SURVEY, DriveSubmit.TASK_2_TRACKING])
        self.declare_parameter('boundary.lat', [0.0])
        self.declare_parameter('boundary.lon', [0.0])

        self.team = self.get_parameter('team_name').value
        self.folder_id = self.get_parameter('folder_id').value.strip()
        self.templates = {DriveSubmit.TASK_1_SURVEY: TASK1_NAME_TEMPLATE,
                          DriveSubmit.TASK_2_TRACKING: self.get_parameter('task2_name_template').value}
        self.tasks = set(self.get_parameter('tasks').value) & set(TASK_LABELS)
        lat, lon = self.get_parameter('boundary.lat').value, self.get_parameter('boundary.lon').value
        self.boundary = list(zip(lat, lon)) if len(lat) == len(lon) >= 3 else None

        for task in sorted(self.tasks):
            try:
                self.get_logger().info(f'{TASK_LABELS[task]} : remise sous le nom {remote_name(self.templates[task], self.team)}.')
            except ValueError as e:
                self.get_logger().error(f'{TASK_LABELS[task]} : aucune remise possible, {e}.')
        credentials = [os.environ.get(name, '').strip() for name in CREDENTIAL_ENVS]
        self.drive = Drive(*credentials) if all(credentials) and self.folder_id else None
        if self.drive is None:
            self.get_logger().warn(f'Identifiants Drive ({", ".join(CREDENTIAL_ENVS)} dans .env) ou folder_id '
                                   '(config/drive_uploader.yaml) manquants : validation seulement.')

        self.status_pub = self.create_publisher(DriveStatus, topics.DRIVE_STATUS, 10)
        self.create_subscription(DriveSubmit, topics.DRIVE_SUBMIT, self.on_submit, 10)
        self.requests = queue.Queue()
        self.events = queue.Queue()          # ('status', DriveStatus) ou (niveau, texte), du thread de travail
        self.create_timer(EVENT_PERIOD_S, self.drain)
        threading.Thread(target=self.work, daemon=True).start()

    def on_submit(self, msg):
        if msg.task in self.tasks:           # sinon l'autre instance (sol ou drone) s'en charge
            self.requests.put(msg)

    def drain(self):
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == 'status':
                value.header.stamp = self.get_clock().now().to_msg()
                self.status_pub.publish(value)
            elif kind == 'info':             # une ligne par sévérité : rclpy refuse deux sévérités au même appel
                self.get_logger().info(value)
            elif kind == 'warn':
                self.get_logger().warn(value)
            else:
                self.get_logger().error(value)

    # ----- thread de travail

    def publish(self, status, state, message):
        status.state, status.message = state, message
        out = DriveStatus()                  # une copie : status continue de changer dans ce thread
        for name in DriveStatus.get_fields_and_field_types():
            setattr(out, name, getattr(status, name))
        self.events.put(('status', out))

    def work(self):
        while True:
            msg = self.requests.get()
            try:
                self.process(msg)
            except Exception as e:           # le thread ne doit jamais mourir : la demande suivante doit passer
                self.events.put(('error', f'Demande {msg.request_id} : erreur inattendue {e!r}.'))

    def process(self, msg):
        label = TASK_LABELS[msg.task]
        s = DriveStatus(request_id=msg.request_id, task=msg.task, path=msg.path)
        errors = []                          # le fichier est validé même sans nom d'équipe : utile en essai
        try:
            s.remote_name = remote_name(self.templates[msg.task], self.team)
        except ValueError as e:
            errors.append(f'{e}.')
        try:
            with open(msg.path, 'rb') as f:
                data = f.read()
        except OSError as e:
            s.errors = errors + [f'Lecture de {msg.path} impossible : {e}.']
            self.publish(s, DriveStatus.STATE_INVALID, f'{label} : fichier illisible.')
            return

        report = validate(msg.task, data, self.boundary)
        s.errors, s.warnings, s.summary = errors + report.errors, report.warnings, report.summary
        s.md5 = hashlib.md5(data).hexdigest()
        if s.errors:
            self.publish(s, DriveStatus.STATE_INVALID, f'{label} : {len(s.errors)} erreur(s), rien n\'est remis.')
            return
        if not msg.upload:
            self.publish(s, DriveStatus.STATE_VALID, f'{label} : prêt à remettre, {len(s.warnings)} avertissement(s).')
            return
        if s.warnings and not msg.force:
            self.publish(s, DriveStatus.STATE_NEEDS_FORCE, f'{label} : {len(s.warnings)} avertissement(s), confirmer avec force.')
            return
        if self.drive is None:
            self.publish(s, DriveStatus.STATE_FAILED, f'{label} : identifiants Drive ou folder_id manquants, rien n\'est remis.')
            return

        self.publish(s, DriveStatus.STATE_UPLOADING, f'{label} : envoi de {s.remote_name}.')
        try:
            meta, s.replaced = self.drive.upload(self.folder_id, s.remote_name, data)
        except DriveError as e:
            self.publish(s, DriveStatus.STATE_FAILED, f'{label} : pas remis, {e}.')
            self.events.put(('error', f'{label} : {s.remote_name} pas remis, {e}.'))
            return
        s.drive_file_id, s.drive_link, s.drive_modified_time = meta['id'], meta.get('webViewLink', ''), meta['modifiedTime']
        done = f'{s.remote_name} {"remplacé" if s.replaced else "remis"} à {s.drive_modified_time} (heure Drive)'
        self.publish(s, DriveStatus.STATE_UPLOADED, f'{label} : {done}.')
        self.events.put(('info', f'{label} : {done}, md5 vérifié{", forcé malgré des avertissements" if s.warnings else ""}.'))


def main(args=None):
    rclpy.init(args=args)
    node = DriveUploader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
