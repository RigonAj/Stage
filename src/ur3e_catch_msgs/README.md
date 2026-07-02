# ur3e_catch_msgs

Paquet de **messages typés et horodatés** du contrat de catch
(`docs/Robot_Control/ur3e_live_catch_architecture.md` §5).

Type de paquet : **rosidl / `ament_cmake`**.

## Messages prévus (`msg/`)

- `BallState.msg` — pose balle horodatée (stamp = temps d'événement ;
  `frame_id` = repère déclaré : `<camera_frame>` ou `base_link`), position en mètres,
  vitesse optionnelle, `valid`, `confidence`.
- `CatchTelemetry.msg` — debug/visualisation hors chemin critique
  (observation 33-D, action brute 6, cible articulaire 6, balle en frame policy `base_link`).

## À ajouter à l'implémentation

`package.xml`, `CMakeLists.txt` (avec `rosidl_generate_interfaces`), puis les
`.msg` dans `msg/`. Doit être buildable et visible par `ball_tracking_cpp`
(producteur) et `ur3e_live_catch` (consommateur) — workspace ROS unique (§3).
