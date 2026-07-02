# CatchTelemetry.msg — debug / visualisation, HORS chemin critique 60 Hz (archi §5).
float32[]            observation     # 33 (ordre de firsttraining_env, archi §6)
float32[]            raw_action      # 6  (sortie policy, avant mapping/clip)
float32[]            joint_target    # 6  cible sûre (après clip + rate + accel-limit) ;
                                     #    renseignée même en dry-run (cible NON émise)
geometry_msgs/Point  ball_base       # balle dans le frame policy base_link (nom conservé ABI)
geometry_msgs/Vector3 ball_vel_base  # vitesse filtrée en base_link/world local (nom conservé ABI)
# --- budget de latence (archi §10, étape 8) ---
float32              perception_age_s  # now - BallState.stamp au tick (âge perception)
float32              loop_compute_s    # temps de calcul du chemin chaud sur ce tick
# --- mode commande (étape 9 / UI Test) ---
bool                 command_enabled   # true => le nœud émet des commandes robot (sinon dry-run)
