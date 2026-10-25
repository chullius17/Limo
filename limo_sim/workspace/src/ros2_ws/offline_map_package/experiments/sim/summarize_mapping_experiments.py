#!/usr/bin/env python3
"""Index completed, comparable reconstruction reports without assigning accuracy scores."""

import argparse
import csv
import json
from pathlib import Path


DIRECTORY = Path(__file__).resolve().parent


def summarize(directory):
    experiments = []
    for path in directory.glob('*/*_report.json'):
        report = json.loads(path.read_text(encoding='utf-8'))
        config = report.get('effective_mapper_config', report['recorded_mapper_config'])
        name = path.name[:-len('_report.json')]
        experiments.append((path.parent.name, name, report, config))
    if not experiments:
        raise ValueError('No experiment reports found in ' + str(directory))
    experiments.sort(key=lambda item: (
        item[2]['temporal_filter'],
        -item[3]['log_odds_limit'] if item[2]['temporal_filter'] else 0, item[0]))
    baseline = experiments[0][2]
    for case, _, report, _ in experiments:
        for key in ('bag_sha256', 'statistics', 'geometry'):
            if report[key] != baseline[key]:
                raise ValueError('Experiment {} differs in {}'.format(case, key))

    rows = []
    markdown = [
        '# Esperimenti di mapping — ' + directory.name, '',
        'Ogni cartella contiene PGM e YAML raw, anteprime PNG, dati NPZ, '
        'parametri YAML e report JSON. Le mappe `_semantic` escludono '
        'la sovrapposizione laser.', '',
        'Tutte le prove usano lo stesso rosbag, le stesse TF, la stessa geometria '
        'e **{} cloud integrate**.'.format(baseline['statistics']['clouds_integrated']), '',
        '| Esperimento | hit | miss | limite | soglia | '
        'Sconosciute tra le osservate | Anteprima semantica |',
        '| --- | ---: | ---: | ---: | ---: | ---: | --- |',
    ]
    for case, name, report, config in experiments:
        filtered = report['temporal_filter']
        observed = report['observed_cells']
        unknown = report['observed_but_unknown_cells']
        percent = 100.0 * unknown / observed if observed else 0.0
        costs = report['semantic_cost_counts']
        row = {
            'experiment': case, 'temporal_filter': filtered,
            'hit': config['hit_log_odds'] if filtered else '',
            'miss': config['miss_log_odds'] if filtered else '',
            'hit_plus_miss': (config['hit_log_odds'] + config['miss_log_odds']
                              if filtered else ''),
            'positive_update_fraction_threshold': (
                config['miss_log_odds'] / (config['hit_log_odds'] + config['miss_log_odds'])
                if filtered else ''),
            'limit': config['log_odds_limit'] if filtered else '',
            'threshold': config['min_evidence'] if filtered else '',
            'free_confirmations': config.get('free_confirmations', 1) if filtered else '',
            'clouds_integrated': report['statistics']['clouds_integrated'],
            'observed_cells': observed, 'unknown_observed': unknown,
            'unknown_observed_percent': round(percent, 4),
            'semantic_road_cells': costs.get('0', 0),
            'semantic_white_cells': costs.get(str(config['white_cost']), 0),
            'semantic_yellow_cells': costs.get(str(config['yellow_cost']), 0),
            'semantic_boardwalk_cells': costs.get(str(config['boardwalk_cost']), 0),
        }
        rows.append(row)
        markdown.append(
            '| [{case}]({case}/) | {hit} | {miss} | {limit} | {threshold} | '
            '{unknown} / {observed} ({percent:.2f}%) | [PNG]({case}/{name}_semantic.png) |'.format(
                case=case, name=name, hit=row['hit'] if filtered else '—',
                miss=row['miss'] if filtered else '—',
                limit=row['limit'] if filtered else '—',
                threshold=row['threshold'] if filtered else '—',
                unknown=unknown, observed=observed, percent=percent))
    markdown.extend([
        '', '## Interpretazione', '',
        'Il conteggio delle celle sconosciute misura la copertura classificata, '
        'non l’accuratezza. Il confronto con la mappa ideale richiede un allineamento '
        'comune dei frame e metriche per classe, ancora da calcolare.', '',
        'Nelle prove hit/miss la saturazione resta 3 e hit + miss resta 1,25. '
        'L’aggiornamento di una classe è positivo quando la sua frazione di punti '
        'supera miss / (hit + miss): 32% per 0,85/0,40, 20% per 1,00/0,25 '
        'e 50% per 0,625/0,625. Le prove con saturazione 1 e 2 restano archiviate.', '',
        'La ricostruzione è offline: consulta la storia TF completa ai timestamp '
        'dei sensori e non simula perdite delle code o timeout del replay in tempo reale.', '',
        '## Riproduzione', '',
        'Nel container con ROS configurato, eseguire '
        '`reconstruct_filtered_map.py --limit 2` oppure `--limit 1`. '
        'Lo script seleziona automaticamente la cartella e il nome della variante. '
        'Il profilo di mapping originale non viene modificato.', '',
        'Per le prove hit/miss usare `reconstruct_filtered_map.py --limit 3 '
        '--hit 1 --miss 0.25` oppure `--limit 3 --hit 0.625 --miss 0.625`.', '',
        'Rigenerare questo riepilogo con `python3 summarize_mapping_experiments.py`.', '',
    ])
    (directory / 'README.md').write_text('\n'.join(markdown), encoding='utf-8')
    with (directory / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, default=DIRECTORY / 'results/run_01')
    args = parser.parse_args()
    for row in summarize(args.results):
        print('{}: {} unknown / {} observed ({:.2f}%)'.format(
            row['experiment'], row['unknown_observed'], row['observed_cells'],
            row['unknown_observed_percent']))


if __name__ == '__main__':
    main()
