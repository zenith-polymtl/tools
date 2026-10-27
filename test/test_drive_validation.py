"""Validators of the Phase 2 hand-in (tools/drive_uploader.py).

Exception to the « no tests » rule of ARCHITECTURE.md, decided for drive_uploader: a wrong file handed
in costs points that cannot be recovered. Run: `colcon test --packages-select tools` in a
built workspace (the module imports rclpy and custom_interfaces).
"""
from datetime import datetime, timedelta, timezone

import pytest

from tools import drive_uploader as v

GOOD_T1 = b'Total deer: 8\nDeer per cluster (10 m radius): 2, 2, 3, 1\nTag IDs: H7, K4, 3P\nAnomalous deer: One deer is wearing a hat.\n'
# Limite de vol de Medicine Hat (CONOPS, Annexe C)
BOUNDARY = list(zip([50.0970884, 50.1061216, 50.1061482, 50.1035232, 50.0988785, 50.0971194],
                    [-110.7328077, -110.7327756, -110.7437887, -110.7437798, -110.7382540, -110.7382533]))
START = datetime(2027, 5, 16, 15, 0, 0, tzinfo=timezone.utc)


def track(seconds=301, step=1.0, speed=2.0, lat0=50.1000, lon0=-110.7350, stamp=None):
    """A CSV going north at `speed` m/s, one point every `step` s."""
    lines = ['timestamp,latitude,longitude']
    t = 0.0
    while t <= seconds:
        when = START + timedelta(seconds=t)
        text = stamp(when) if stamp else when.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
        lines.append(f'{text},{lat0 + speed * t / 111_195:.7f},{lon0:.7f}')
        t += step
    return ('\n'.join(lines) + '\n').encode()


def t2(data, boundary=BOUNDARY):
    return v.validate_task2(data, boundary)


# ---------------------------------------------------------------- noms

def test_remote_names():
    assert v.remote_name(v.TASK1_NAME_TEMPLATE, 'Zenith') == 'Zenith_task1_survey.txt'
    assert v.remote_name('{team}_task2_tracking.csv', ' Zenith ') == 'Zenith_task2_tracking.csv'
    for bad in ('', '  ', 'a/b', "o'k"):
        with pytest.raises(ValueError):
            v.remote_name(v.TASK1_NAME_TEMPLATE, bad)
    with pytest.raises(ValueError):
        v.remote_name('task2.csv', 'Zenith')


# ---------------------------------------------------------------- Tâche 1

def test_t1_good():
    r = v.validate_task1(GOOD_T1)
    assert r.ok and not r.warnings
    assert r.summary == '8 cerfs, 4 groupes (2, 2, 3, 1), étiquettes : H7, K4, 3P'


def test_t1_format_helper_round_trip():
    assert v.format_task1_report(8, [2, 2, 3, 1], ['H7', 'K4', '3P'], 'One deer is wearing a hat.').encode() == GOOD_T1
    r = v.validate_task1(v.format_task1_report(0, [], [], '').encode())
    assert r.ok and not r.warnings


def test_t1_labels_case_crlf_blank_lines_bom():
    r = v.validate_task1(b'\xef\xbb\xbftotal DEER: 1\r\n\r\ndeer per cluster (10 m radius): 1\r\ntag ids: none\r\nanomalous deer: none observed\r\n')
    assert r.ok and not r.warnings


@pytest.mark.parametrize('data, needle', [
    (b'', 'vide'),
    (b'\xff\xfeT\x00', 'UTF-8'),
    (GOOD_T1.replace(b'Tag IDs: H7, K4, 3P\n', b''), 'Tag IDs'),
    (GOOD_T1 + b'Total deer: 9\n', 'deux fois'),
    (GOOD_T1.replace(b'Total deer: 8', b'Total deer: eight'), 'entier'),
    (GOOD_T1.replace(b'2, 2, 3, 1', b'2, 0, 3'), 'entiers'),
    (GOOD_T1.replace(b'H7, K4', b'h7, K4'), 'majuscules'),
    (GOOD_T1.replace(b'H7, K4', b'H77, K4'), 'majuscules'),
    (GOOD_T1.replace(b'H7, K4', b'H7 K4'), 'majuscules'),
])
def test_t1_errors(data, needle):
    r = v.validate_task1(data)
    assert not r.ok
    assert any(needle in e for e in r.errors), r.errors


@pytest.mark.parametrize('data, needle', [
    (GOOD_T1.replace(b'Total deer: 8', b'Total deer: 9'), 'groupes font 8'),
    (GOOD_T1.replace(b'K4, 3P', b'H7, 3P'), 'double'),
    (GOOD_T1.replace(b'One deer is wearing a hat.', b''), 'vide'),
    (GOOD_T1 + b'Weather: windy\n', 'non reconnue'),
    (b'Total deer: 1\nDeer per cluster (10 m radius): 1\nTag IDs: A1, B2\nAnomalous deer: none observed\n', '2 étiquettes pour 1'),
])
def test_t1_warnings(data, needle):
    r = v.validate_task1(data)
    assert r.ok, r.errors
    assert any(needle in w for w in r.warnings), r.warnings


# ---------------------------------------------------------------- Tâche 2

def test_t2_appendix_d_is_valid():
    """L'exemple de l'Annexe D : format valide ; trop court et trop rapide, donc des avertissements."""
    data = (b'timestamp,latitude,longitude\n'
            b'2026-07-31T22:33:02.000Z,50.0970884,-110.7328077\n'
            b'2026-07-31T22:33:03.123Z,50.1061216,-110.7327756\n'
            b'2026-07-31T22:33:04.555Z,50.1061482,-110.7437887\n'
            b'2026-07-31T22:33:08.612Z,50.1035232,-110.7437798\n')
    r = v.validate_task2(data, BOUNDARY)
    assert r.ok, r.errors
    assert any('couvre 7 s' in w for w in r.warnings)
    assert any('m/s' in w for w in r.warnings)


def test_t2_good():
    r = t2(track())
    assert r.ok and not r.warnings, (r.errors, r.warnings)
    assert r.summary.startswith('302 points, 15:00:00 à 15:05:01 UTC, 301 s')


@pytest.mark.parametrize('stamp', [
    lambda t: t.strftime('%Y-%m-%dT%H:%M:%SZ'),
    lambda t: t.strftime('%Y-%m-%dT%H:%M:%S.%f+00:00'),
    lambda t: (t - timedelta(hours=6)).strftime('%Y-%m-%dT%H:%M:%S-06:00'),
    lambda t: t.strftime('%Y%m%dT%H%M%SZ'),
    lambda t: t.strftime('"%Y-%m-%dT%H:%M:%S,%fZ"'),   # virgule décimale : valide entre guillemets seulement
])
def test_t2_iso8601_variants(stamp):
    r = t2(track(stamp=stamp))
    assert r.ok and not r.warnings, (r.errors, r.warnings)


def test_t2_offset_is_applied():
    assert v.parse_iso8601('2027-05-16T09:00:00-06:00')[0] == datetime(2027, 5, 16, 15, 0, tzinfo=timezone.utc)
    assert v.parse_iso8601('2027-05-16T15:00:00.5Z')[0] == datetime(2027, 5, 16, 15, 0, 0, 500000, tzinfo=timezone.utc)
    assert v.parse_iso8601('2027-02-30T15:00:00Z') is None
    assert v.parse_iso8601('2027-05-16 15:00:00Z') is None


@pytest.mark.parametrize('data, needle', [
    (b'', 'vide'),
    (b'timestamp,latitude,longitude\n', 'Aucun point'),
    (track().split(b'\n', 1)[1], "l'en-tête"),
    (track().replace(b'timestamp,latitude,longitude', b'timestamp,lat'), 'En-tête'),
    (track().replace(b'2027-05-16T15:00:10.000Z', b'2027-05-16 15:00:10'), 'ISO 8601'),
    (track().replace(b'2027-05-16T15:00:10.000Z', b'1778943610'), 'ISO 8601'),
    (track().replace(b'2027-05-16T15:00:10.000Z', b'2027-05-16T15:00:09.000Z'), 'égal'),
    (track().replace(b'2027-05-16T15:00:10.000Z', b'2027-05-16T15:00:05.000Z'), 'antérieur'),
    (track().replace(b'-110.7350000', b'-110.7350000,0', 1), '3 colonnes'),
    (track().replace(b'50.1000000,', b'50 6.0N,', 1), 'degrés décimaux'),
    (track().replace(b'50.1000000,-110.7350000', b'-110.7350000,50.1000000', 1), 'hors limites'),
    (track().replace(b'2027-05-16T15:00:10.000Z', b'2027-05-16T15:00:10.000'), 'certaines lignes'),
])
def test_t2_errors(data, needle):
    r = t2(data)
    assert not r.ok
    assert any(needle in e for e in r.errors), r.errors


def test_t2_errors_are_capped():
    data = b'timestamp,latitude,longitude\n' + b'bad,1,2\n' * 50
    r = t2(data)
    assert len(r.errors) == 11


@pytest.mark.parametrize('data, needle', [
    (track(seconds=200), 'couvre 200 s'),
    (track(step=0.1, speed=9.0), 'plus de 7.5 m/s'),
    (track(lat0=50.0960), 'hors de la limite'),
    (b'\n'.join(l for l in track().split(b'\n') if not l.startswith(b'2027-05-16T15:02:'))
     , 'trou'),
    (track(stamp=lambda t: t.strftime('%Y-%m-%dT%H:%M:%S')), 'sans fuseau'),
])
def test_t2_warnings(data, needle):
    r = t2(data)
    assert r.ok, r.errors
    assert any(needle in w for w in r.warnings), r.warnings


def test_t2_gps_noise_at_high_rate_is_not_a_speed_warning():
    """10 Hz et 1 m de bruit : de point en point plus de 10 m/s, sur 2 s une vitesse réaliste."""
    lines = track(step=0.1, speed=3.0).split(b'\n')
    noisy = [lines[0]] + [l.replace(b',-110.7350000', b',-110.7350140') if i % 2 else l
                          for i, l in enumerate(lines[1:])]
    r = t2(b'\n'.join(noisy))
    assert r.ok and not any('m/s' in w for w in r.warnings), r.warnings



def test_t2_without_boundary_skips_that_check():
    r = v.validate_task2(track(lat0=50.0960), None)
    assert r.ok and not r.warnings


def test_point_in_non_convex_boundary():
    assert v.inside(50.1050, -110.7400, BOUNDARY)        # nord-ouest, dedans
    assert not v.inside(50.0975, -110.7420, BOUNDARY)    # le coin coupé au sud-ouest
    assert not v.inside(50.0960, -110.7350, BOUNDARY)    # au sud de la ligne de vol
