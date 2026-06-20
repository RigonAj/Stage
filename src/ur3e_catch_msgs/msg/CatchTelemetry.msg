# CatchTelemetry.msg — debug / visualisation, HORS chemin critique 60 Hz (archi §5).
float32[]            observation   # 33 (ordre de firsttraining_env, archi §6)
float32[]            raw_action    # 6  (sortie policy, avant mapping/clip)
float32[]            joint_target  # 6  (après clip + rate-limit ; VIDE en dry-run)
geometry_msgs/Point  ball_base     # balle dans le repère base
