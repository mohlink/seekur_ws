# low_level/ — Diagnostic série bas niveau

## seekur_baudrate_detector.cpp

Utilitaire C++ qui teste séquentiellement plusieurs baudrates (9600, 19200,
38400, 57600, 115200) sur un port série donné, envoie des trames SYNC
SeekurOS, et rapporte lequel provoque une réponse cohérente du robot.

Utilisé une fois pour confirmer que le SeekurJR utilise **9600 baud** par
défaut. Documenté ici pour le jour où on rebranche un vrai robot dont la
config n'est plus certaine (unité neuve, configuration modifiée par un
utilisateur précédent, câble USB-série suspecté).

### Compilation

    make

### Utilisation

    ./seekur_baudrate_detector /dev/ttyUSB0

Le programme itère sur les baudrates et affiche celui qui fonctionne.

## Note historique

Un `seekur_minimal_driver.cpp` a existé dans les premières phases du projet
(driver C++ complet en ligne de commande), mais n'a jamais reçu la gestion
des signaux DTR/RTS pourtant nécessaires pour que le firmware SeekurOS
réponde. La logique validée est passée intégralement côté Python
(`tools/interactive/`) et n'a pas été portée en C++. Le driver C++ reste
dans `~/seekurJr/claude/arch/` à titre d'archive, non versionné.
