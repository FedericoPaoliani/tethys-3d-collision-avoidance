# Risorse locali e riferimento upstream

Questa è una selezione del modello Tethys usata nel progetto di Federico Paoliani. `model.sdf` è la versione adattata localmente; i sei mondi contengono ulteriori configurazioni per sonar, attuatori e prove. Il file `model.sdf.in` resta il template upstream incluso nella raccolta.

Confronto con osrf/lrauv commit `8cf77021c2e6b6727592f1d952c97349823ecc94`. Una differenza da questo commit non prova da sola una modifica locale, perché il commit di origine della raccolta non è noto.

| File | Confronto |
|---|---|
| `CMakeLists.txt` | Identico salvo fine riga |
| `colcon.pkg` | Identico salvo fine riga |
| `hooks/hook.dsv.in` | Identico salvo fine riga |
| `hooks/hook.sh.in` | Identico salvo fine riga |
| `scripts/description_generator.py` | Identico salvo fine riga |
| `models/tethys/model.config` | Identico salvo fine riga |
| `models/tethys/model.sdf` | Solo nella raccolta locale |
| `models/tethys/model.sdf.in` | Identico |
| `models/tethys/meshes/tethys.dae` | Identico salvo fine riga |
| `models/tethys/materials/scripts/model.material` | Identico |
| `models/tethys/materials/textures/Tethys_Albedo.png` | Identico |
| `models/tethys/materials/textures/Tethys_Metalness.png` | Identico |
| `models/tethys/materials/textures/Tethys_Normal.png` | Identico |
| `models/tethys/materials/textures/Tethys_Roughness.png` | Identico |
