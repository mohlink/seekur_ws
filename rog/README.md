# ROG-Strix (poste de dev) — configuration systeme

## CycloneDDS
Fichier actif : `~/cyclonedds.xml`, charge par `~/.bashrc` :

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

Interfaces autorisees : `wlo1` (WiFi) et `enp108s0` (Ethernet), pas `tailscale0`.
Copie de reference : `rog/config/cyclonedds.xml`. Restauration :

    cp rog/config/cyclonedds.xml ~/cyclonedds.xml
