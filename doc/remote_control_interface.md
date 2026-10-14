# Les interrupteurs de la manette : `remote_control_interface`

Ce node fait le lien entre les interrupteurs de la manette du pilote et le code de mission. Il sert à bouger un servo (caméra, pince, verrou) ou à lancer une action (le GO, un largage, une photo) d'un geste sur la manette.

On ne touche jamais au code du node. Tout se règle dans `config/<mission>.yaml`, section `remote_control_interface`.

## Comment ça marche

```
 MANETTE                  ROS                      LE NODE                        EFFET
┌─────────┐           /mavros/rc/in      ┌───────────────────────────┐
│ inter-  │  radio   (16 canaux, un PWM  │ 1. lit le PWM du canal     │
│ rupteur │ ───────► par canal, 1000 à   │ 2. le convertit en position│
│ 3 pos.  │          2000)               │    PWM < 1300   → LOW      │
└─────────┘                              │    1300 à 1700  → MIDDLE   │
                                         │    PWM > 1700   → HIGH     │
                                         │ 3. si la position a changé,│
                                         │    fait ce que le yaml dit │
                                         └────────────┬──────────────┘
                                                      │
                          ┌───────────────────────────┼───────────────────────────┐
                          ▼                           ▼                           ▼
                   servo + pwm                     events                  events + trigger
                 bouge un servo              publie un Bool True        publie un Bool True, mais
                 (caméra, pince)             sur un topic (le GO)       seulement quand un AUTRE
                                                                        interrupteur le déclenche
```

Le node ne connaît que trois positions par interrupteur : **LOW, MIDDLE, HIGH**. Dans le yaml, toute liste de trois valeurs suit cet ordre : `[LOW, MIDDLE, HIGH]`.

## Ajouter un interrupteur en trois étapes

1. **Trouver son canal.** Dans EdgeTX, ou avec `make echo T=/mavros/rc/in` en bougeant l'interrupteur : la valeur qui change est la bonne. Le yaml compte à partir de 0 : le canal 8 de la manette s'écrit `channel: 7`.
2. **Lui donner un nom** et l'ajouter à la liste `switches`.
3. **Écrire sa section** sous ce nom : ce qu'il doit faire, avec l'un des trois cas ci-dessous.

## Cas 1 : bouger un servo

L'interrupteur du canal 9 ouvre et ferme la pince, branchée sur le servo 10.

```yaml
remote_control_interface:
  ros__parameters:
    switches: [pince]
    pince:
      channel: 8                  # canal 9 de la manette
      servo: 10                   # numéro du servo à commander
      pwm: [1100, 1500, 1900]     # PWM envoyé au servo en LOW, MIDDLE, HIGH
```

```
 interrupteur en bas    → servo 10 à 1100 (fermé)
 interrupteur au milieu → servo 10 à 1500
 interrupteur en haut   → servo 10 à 1900 (ouvert)
```

Pour un interrupteur à deux positions, on répète une valeur : `pwm: [1100, 1900, 1900]`.

Le node ne pilote pas le servo lui-même : il le demande au service `topics.PAYLOAD_SET_STATE` (`custom_interfaces/srv/ServoState`), fourni par le node payload de la mission. Sans ce node, il écrit un WARN et le servo ne bouge pas.

## Cas 2 : lancer une action

Mettre l'interrupteur du canal 10 en haut envoie le GO de la mission.

```yaml
    switches: [go]
    go:
      channel: 9
      events: ['', '', MISSION_GO]    # rien en LOW, rien en MIDDLE, le GO en HIGH
```

Quand l'interrupteur **arrive** en HIGH, le node publie `std_msgs/Bool` (`data: true`) sur le topic de `MISSION_GO`. On écrit le nom de la constante de `tools/topics.py`, jamais le texte du topic. `''` veut dire « rien ».

### Créer une action qui n'existe pas encore

Exemple : larguer le tracker. Il faut trois choses.

1. **Le nom du topic**, une ligne dans `tools/topics.py` :
   ```python
   TRACKER_RELEASE = f'{INTERNAL}/tracker/release'   # Bool, le pilote demande le largage
   ```
2. **Le branchement à la manette**, dans le yaml : `events: ['', '', TRACKER_RELEASE]`.
3. **Le node qui fait le travail**, abonné à ce topic. `remote_control_interface` ne fait que dire « le pilote a demandé ça » ; c'est ce node qui agit :
   ```python
   from std_msgs.msg import Bool
   from tools import topics

   self.create_subscription(Bool, topics.TRACKER_RELEASE, self.on_release, 10)

   def on_release(self, msg):
       self.get_logger().info('Largage du tracker demandé par le pilote')
       ...
   ```

```
 manette ──► remote_control_interface ──► /aeac/internal/tracker/release ──► node tracker ──► largage
             (écrit dans le yaml)          (écrit dans topics.py)              (ton code)
```

## Cas 3 : choisir une action, puis la déclencher

Un interrupteur 3 positions **choisit** l'action, un autre la **lance**. Un interrupteur accroché par erreur ne déclenche donc rien tout seul. C'était le système de la compétition 2026.

```yaml
    switches: [choix, feu]
    choix:
      channel: 6
      events: [PHOTO, SAMPLE, TRACKER_RELEASE]   # l'action choisie selon la position
      trigger: feu                               # attend l'interrupteur « feu »
    feu:
      channel: 7                                 # il ne fait rien d'autre
```

```
   interrupteur « choix »            interrupteur « feu »
   ┌──────┐                          ┌──────┐
   │ HIGH │ TRACKER_RELEASE          │ HIGH │ ── passe en HIGH ──► publie l'action
   │ MID  │ SAMPLE  ◄── position     │      │                     choisie à cet instant
   │ LOW  │ PHOTO       actuelle     │ LOW  │                     (ici SAMPLE)
   └──────┘                          └──────┘
   bouger « choix » ne publie rien   ramener « feu » en bas ne publie rien
```

Par défaut, le déclenchement se fait quand `feu` passe en HIGH. Pour une autre position : `trigger_position: MIDDLE` dans la section `choix`.

`PHOTO`, `SAMPLE` et `TRACKER_RELEASE` sont des exemples : ils doivent exister dans `topics.py` (voir le cas 2).

## Tout ensemble

Plusieurs interrupteurs, chacun sa section. Un même interrupteur peut bouger un servo **et** lancer une action : `servo`, `pwm` et `events` dans la même section.

```yaml
remote_control_interface:
  ros__parameters:
    threshold_low: 1300
    threshold_high: 1700
    switches: [pince, go, choix, feu]
    pince: {channel: 8, servo: 10, pwm: [1100, 1500, 1900]}
    go:    {channel: 9, events: ['', '', MISSION_GO]}
    choix: {channel: 6, events: [PHOTO, SAMPLE, TRACKER_RELEASE], trigger: feu}
    feu:   {channel: 7}
```

## À savoir

- **Au démarrage**, le node lit la position de chaque interrupteur. Il règle les servos sur cette position, mais ne lance aucune action, même si un interrupteur est déjà en haut. Il faut le bouger.
- **Une action part au changement de position**, pas tant que l'interrupteur y reste. Laisser `go` en HIGH n'envoie le GO qu'une fois.
- **Le node ne change jamais de mode de vol et n'arme jamais.** C'est le pilote (règle de `ARCHITECTURE.md`).
- **Les mêmes canaux dans deux missions ne posent pas de problème** : chaque mission a son yaml.

## Tester sans drone

```
make sim C=hexa    # terminal 1 : la mission, donc ce node
make rc            # terminal 2 : le clavier simule la manette
```

`make rc` ne simule que les canaux 8, 9 et 10 de la manette, soit `channel: 7, 8, 9` dans le yaml :

| Touches | Canal manette | `channel:` | Haut / centre / bas |
|---|---|---|---|
| `w s x` | 8 | 7 | `w` / `s` / `x` |
| `e d c` | 9 | 8 | `e` / `d` / `c` |
| `r f v` | 10 | 9 | `r` / `f` / `v` |

Les logs du node disent ce qui se passe : `go : HIGH`, puis `go : publication sur /aeac/external/mission/go`. Pour voir le topic lui-même : `make echo T=/aeac/external/mission/go`.

## Tous les réglages

| Réglage | Où | Obligatoire | Rôle |
|---|---|---|---|
| `threshold_low` | section du node | non (1300) | PWM sous lequel la position est LOW |
| `threshold_high` | section du node | non (1700) | PWM au-dessus duquel la position est HIGH |
| `switches` | section du node | non | Les noms des interrupteurs utilisés ; sans lui, le node ne fait rien |
| `channel` | section d'un interrupteur | oui | Index dans `RCIn.channels` (0 = canal 1) |
| `servo` | section d'un interrupteur | non | Numéro du servo à commander ; absent = pas de servo |
| `pwm` | section d'un interrupteur | si `servo` | Trois PWM : LOW, MIDDLE, HIGH |
| `events` | section d'un interrupteur | non | Trois constantes de `tools/topics.py` (ou `''`) : LOW, MIDDLE, HIGH |
| `trigger` | section d'un interrupteur | non | Nom de l'interrupteur qui déclenche ; demande `events` |
| `trigger_position` | section d'un interrupteur | non (HIGH) | Position du déclencheur qui lance l'action |

## Quand ça ne marche pas

| Message ou symptôme | Cause |
|---|---|
| `Config invalide : interrupteur X : channel manquant` | La section de X n'a pas de `channel`, ou son nom est mal écrit (il doit être identique dans `switches`) |
| `Config invalide : ... n'est pas une constante de tools.topics` | Faute de frappe dans `events`, ou constante pas encore ajoutée à `topics.py` |
| `Config invalide : ... pwm doit donner 3 valeurs` | `servo` est donné, mais `pwm` n'a pas exactement trois valeurs |
| `Config invalide : ... trigger ... absent de switches` | Le déclencheur n'est pas dans la liste `switches` |
| `Interface manette démarrée sans interrupteur` (WARN) | Pas de `switches` dans le yaml : normal tant que la mission n'en a pas |
| `service ... absent, servo non commandé` (WARN) | Le node payload ne tourne pas |
| `déclenché, mais rien à faire pour X en MIDDLE` (WARN) | Le sélecteur est sur une position où `events` vaut `''` |
| Rien ne se passe, aucun log | Mauvais canal : vérifier avec `make echo T=/mavros/rc/in`, et le décalage de 1 |
| Rien ne se passe au démarrage | Normal : il faut bouger l'interrupteur (voir « À savoir ») |
