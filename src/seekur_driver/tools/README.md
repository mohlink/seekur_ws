# tools/ — Outils de troubleshoot et de développement

Ce dossier regroupe des utilitaires **non déployés en production** qui servent
au diagnostic, aux tests manuels et à la validation du protocole SeekurOS,
indépendamment du stack ROS2.

Aucun de ces outils n'est requis pour le fonctionnement du driver
(`seekur_driver_node.py`) ni pour la simulation. Ils existent pour :
- valider une connexion série sur un vrai robot avant d'y attacher ROS2
- envoyer des commandes SeekurOS à la main pour reproduire un bug
- diagnostiquer un port série récalcitrant

## Organisation

- `interactive/` — Contrôleurs Python interactifs qui envoient des commandes
  SeekurOS à un simulateur TCP ou au vrai robot en série. Un fichier par
  contexte d'usage, voir les docstrings.

- `low_level/` — Outils C++ de diagnostic bas niveau. Un seul utilitaire
  validé aujourd'hui (`seekur_baudrate_detector`), voir son README.

## Quand les utiliser

En développement normal, jamais. On lance la simulation avec
`sim_rtabmap.launch.py` (ou autre variante) et on interagit via nav2 ou
`teleop_twist_keyboard`.

Ces outils deviennent utiles quand quelque chose ne marche pas au niveau
du protocole ou de la couche série :
- « le robot ne répond pas à SYNC0 »
- « je ne suis pas sûr du baudrate de ce port »
- « je veux envoyer une trame brute non documentée »