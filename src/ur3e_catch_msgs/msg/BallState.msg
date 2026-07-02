# BallState.msg — pose balle horodatée, partagée perception ↔ commande.
#
# stamp    : temps d'ÉVÉNEMENT (pas le temps de réception) — crucial pour la
#            synchronisation et le budget de latence (archi §5, §10).
# frame_id : repère DÉCLARÉ de la position. <camera_frame> pour le tracker ;
#            <camera_frame> OU base_link pour test_ball_node. Le consommateur
#            (ball_frame) transforme vers le frame policy base_link selon ce seul champ.
std_msgs/Header        header
geometry_msgs/Point    position      # mètres
geometry_msgs/Vector3  velocity      # m/s (optionnel ; sinon recalculée côté ball_frame)
bool                   valid         # estimation exploitable ?
float32                confidence    # qualité (résidu/visibilité)
