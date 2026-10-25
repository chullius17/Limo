# Esperimenti di mapping — run_01

Ogni cartella contiene PGM e YAML raw, anteprime PNG, dati NPZ, parametri YAML e report JSON. Le mappe `_semantic` escludono la sovrapposizione laser.

Tutte le prove usano lo stesso rosbag, le stesse TF, la stessa geometria e **6463 cloud integrate**.

| Esperimento | hit | miss | limite | soglia | Sconosciute tra le osservate | Anteprima semantica |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| [unfiltered](unfiltered/) | — | — | — | — | 348 / 38151 (0.91%) | [PNG](unfiltered/map_unfiltered_semantic.png) |
| [filtered_limit3](filtered_limit3/) | 0.85 | 0.4 | 3.0 | 0.5 | 70 / 38151 (0.18%) | [PNG](filtered_limit3/map_filtered_limit3_semantic.png) |
| [filtered_limit3_hit0p625_miss0p625](filtered_limit3_hit0p625_miss0p625/) | 0.625 | 0.625 | 3.0 | 0.5 | 1046 / 38151 (2.74%) | [PNG](filtered_limit3_hit0p625_miss0p625/map_filtered_limit3_hit0p625_miss0p625_semantic.png) |
| [filtered_limit3_hit1_miss0p25](filtered_limit3_hit1_miss0p25/) | 1.0 | 0.25 | 3.0 | 0.5 | 236 / 38151 (0.62%) | [PNG](filtered_limit3_hit1_miss0p25/map_filtered_limit3_hit1_miss0p25_semantic.png) |
| [filtered_limit2](filtered_limit2/) | 0.85 | 0.4 | 2.0 | 0.5 | 107 / 38151 (0.28%) | [PNG](filtered_limit2/map_filtered_limit2_semantic.png) |
| [filtered_limit1](filtered_limit1/) | 0.85 | 0.4 | 1.0 | 0.5 | 216 / 38151 (0.57%) | [PNG](filtered_limit1/map_filtered_limit1_semantic.png) |

## Interpretazione

Il conteggio delle celle sconosciute misura la copertura classificata, non l’accuratezza. Il confronto con la mappa ideale richiede un allineamento comune dei frame e metriche per classe, ancora da calcolare.

Nelle prove hit/miss la saturazione resta 3 e hit + miss resta 1,25. L’aggiornamento di una classe è positivo quando la sua frazione di punti supera miss / (hit + miss): 32% per 0,85/0,40, 20% per 1,00/0,25 e 50% per 0,625/0,625. Le prove con saturazione 1 e 2 restano archiviate.

La ricostruzione è offline: consulta la storia TF completa ai timestamp dei sensori e non simula perdite delle code o timeout del replay in tempo reale.

## Riproduzione

Nel container con ROS configurato, eseguire `reconstruct_filtered_map.py --limit 2` oppure `--limit 1`. Lo script seleziona automaticamente la cartella e il nome della variante. Il profilo di mapping originale non viene modificato.

Per le prove hit/miss usare `reconstruct_filtered_map.py --limit 3 --hit 1 --miss 0.25` oppure `--limit 3 --hit 0.625 --miss 0.625`.

Rigenerare questo riepilogo con `python3 summarize_mapping_experiments.py`.
