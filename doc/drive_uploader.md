# drive_uploader : remise des fichiers de la Phase 2 sur le Google Drive

`ros2 run tools drive_uploader`. Tout le code est dans `tools/drive_uploader.py`.

Deux fichiers se remettent sur le Google Drive de l'équipe pendant les fenêtres de vol :

| Tâche | Fichier sur le Drive | Produit par | Échéance (CONOPS) |
|---|---|---|---|
| 1 | `<team>_task1_survey.txt` (nom imposé) | `report_builder` | 15 min après la fin de la fenêtre au plus tard ; ×150 % si remis 5 min avant la fin ou plus tôt (§5.2.3, Table 5) |
| 2 | `<team>_task2_tracking.csv` (nom choisi par nous, `task2_name_template`) | `track_csv_writer` | avant la fin de la fenêtre (§5.2.4, point 6e) |

C'est l'heure d'upload que donne Google Drive qui compte comme heure de remise (§2).

## Comment une node s'en sert

1. La node productrice écrit le fichier, puis publie un `DriveSubmit` sur `topics.DRIVE_SUBMIT` avec `task`, `path` et `upload: false`. Sur `topics.DRIVE_STATUS`, elle reçoit un `DriveStatus` avec le même `request_id` : `VALID` ou `INVALID`, les erreurs, les avertissements et un résumé du contenu. L'UI les montre à l'opérateur.
2. L'opérateur confirme. L'UI publie le même `DriveSubmit` avec `upload: true`, et `force: true` s'il accepte les avertissements. On passe en `UPLOADING`, puis `UPLOADED` : `drive_modified_time` est alors l'heure de remise. Si ça rate, on passe en `FAILED` avec la raison, et on redemande.
3. Une remise refaite remplace le même fichier Drive. Drive garde les versions précédentes, mais **remplacer avance l'heure de remise**.

Les deux topics sont externes. La node tourne normalement au sol (`gcs_bringup`, `hexa.launch.py` et `odos.launch.py`), mais elle peut aussi tourner sur la Jetson. Le paramètre `tasks` dit quelles tâches une instance traite. `path` est un chemin sur la machine de la node.

`report_builder` produit le rapport de la Tâche 1 avec `tools.drive_uploader.format_task1_report(total, clusters, tag_ids, anomaly)`. Pour la Tâche 2, l'en-tête est `CSV_HEADER`.

## Ce qui est vérifié

Une **erreur** bloque la remise, même avec `force`. Un **avertissement** la bloque seulement sans `force`.

- **Tâche 1**, quatre lignes étiquetées. Une liste vide s'écrit `none`.
  ```
  Total deer: 8
  Deer per cluster (10 m radius): 2, 2, 3, 1
  Tag IDs: H7, K4, 3P
  Anomalous deer: One deer is wearing a hat.
  ```
  - Erreurs : une ligne absente ou en double ; un total qui n'est pas un entier ; des groupes qui ne sont pas des entiers ≥ 1 ; une étiquette qui n'a pas exactement 2 caractères `A-Z0-9` en majuscules ; un fichier vide ou pas en UTF-8 ; `team_name` vide.
  - Avertissements : la somme des groupes ne donne pas le total ; étiquettes en double ; plus d'étiquettes que de cerfs ; anomalie vide (écrire `none observed`) ; ligne non reconnue.
- **Tâche 2** : un en-tête, puis `timestamp,latitude,longitude`.
  - Erreurs : pas d'en-tête ; pas exactement 3 colonnes ; horodatage pas en ISO 8601 (avec le `T` : `2027-05-16T15:04:05.123Z`) ; coordonnées pas en degrés décimaux ou hors limites ; ordre chronologique pas strict ; fuseau sur une partie des lignes seulement.
  - Avertissements : moins de 300 s suivies ; trous de plus de 5 s ; plus de 7,5 m/s sur au moins 2 s ; points hors de la limite de vol du site ; horodatages sans fuseau.

## Mise en place

1. Dans la [Google Cloud Console](https://console.cloud.google.com/), avec le compte Google de l'équipe (celui donné aux organisateurs) :
   - créer un projet ;
   - activer l'API Google Drive ;
   - dans l'écran de consentement, choisir le type External ;
   - créer un ID client OAuth de type **Application Web**, avec `https://developers.google.com/oauthplayground` comme URI de redirection autorisée ;
   - **publier l'application** (passer en production). En mode Test, le refresh token expire au bout de 7 jours. L'avertissement « application non validée » est normal.
2. Récupérer le refresh token dans l'[OAuth Playground](https://developers.google.com/oauthplayground) :
   - dans la roue dentée, cocher « Use your own OAuth credentials » et entrer le client ID et le secret ;
   - à l'étape 1, mettre le scope `https://www.googleapis.com/auth/drive` et autoriser avec le compte d'équipe ;
   - à l'étape 2, cliquer « Exchange authorization code for tokens ».
3. Dans le `.env` de chaque machine qui remet : `DRIVE_CLIENT_ID`, `DRIVE_CLIENT_SECRET`, `DRIVE_REFRESH_TOKEN`. Ces valeurs ne vont **jamais dans git ni dans un message**.
4. Dans `config/drive_uploader.yaml` : `team_name`, et `folder_id`, l'id du dossier « Phase 2 Deliverables » (la fin de son URL). Avant d'avoir l'accès, prendre un dossier de test de notre Drive.

Sur la ligne de vol, le portable a besoin de sa propre connexion internet (partage cellulaire, Starlink), à part de la radio du drone.

## Tests

C'est une exception à la règle « pas de tests » d'ARCHITECTURE.md : un fichier mal remis coûte des points qu'on ne rattrape pas. Dans un workspace construit :

```
colcon test --packages-select tools && colcon test-result --verbose
```

## Questions aux organisateurs

- Faut-il un nom précis pour le CSV de la Tâche 2, et va-t-il aussi dans « Phase 2 Deliverables » ?
- Après une re-remise de la Tâche 1, quelle heure compte pour le multiplicateur : la première remise ou la dernière ?
