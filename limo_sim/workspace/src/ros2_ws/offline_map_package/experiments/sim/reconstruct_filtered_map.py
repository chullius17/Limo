#!/usr/bin/env python3
"""Reconstruct the recorded experiment with the current offline temporal filter.

Use the same TF history, cloud selection, final SLAM geometry and export path
as reconstruct_unfiltered_map.py. The filter is the production SemanticGrid,
configured from the current simulation profile. Recorded settings supply any
parameters not explicitly present in that profile. Optional limit, hit and miss
overrides affect this reconstruction only and are included in its output name.
"""

import argparse
import json
import math
from pathlib import Path

import yaml

from reconstruct_unfiltered_map import DIRECTORY, load_config, reconstruct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=DIRECTORY / 'bags/run_01')
    parser.add_argument('--output', type=Path,
                        help='Default: results/<session>/<parameter-based experiment name>')
    parser.add_argument('--name', help='Default: map_<parameter-based experiment name>')
    parser.add_argument('--limit', type=float,
                        help='Override only the evidence saturation for this experiment')
    parser.add_argument('--hit', type=float, help='Override the positive evidence weight')
    parser.add_argument('--miss', type=float, help='Override the negative evidence weight')
    parser.add_argument('--mapping-config', type=Path,
                        default=DIRECTORY.parents[1] / 'config/mapping_sim.yaml')
    args = parser.parse_args()
    with args.mapping_config.open(encoding='utf-8') as stream:
        overrides = yaml.safe_load(stream)['semantic_mapper']
    explicit = {}
    for argument, key in (('limit', 'log_odds_limit'), ('hit', 'hit_log_odds'),
                          ('miss', 'miss_log_odds')):
        value = getattr(args, argument)
        if value is not None:
            if not math.isfinite(value) or value <= 0:
                parser.error('--{} must be finite and positive'.format(argument))
            overrides[key] = value
            explicit[key] = value
    recorded = load_config(args.session.resolve())
    effective = recorded.copy()
    effective.update(overrides)
    limit_label = format(effective['log_odds_limit'], '.6g').replace('.', 'p')
    case = 'filtered_limit' + limit_label
    changed_weights = any(effective[key] != recorded[key]
                          for key in ('hit_log_odds', 'miss_log_odds'))
    if args.hit is not None or args.miss is not None or changed_weights:
        case += '_hit{}_miss{}'.format(
            format(effective['hit_log_odds'], '.6g').replace('.', 'p'),
            format(effective['miss_log_odds'], '.6g').replace('.', 'p'))
    name = args.name if args.name is not None else 'map_' + case
    if not name or Path(name).name != name or name in ('.', '..'):
        parser.error('--name must be a filename stem without a path')
    output = (args.output if args.output is not None
              else DIRECTORY / 'results' / args.session.name / case)
    print('Current temporal parameters: {}'.format({
        key: effective[key] for key in (
            'hit_log_odds', 'miss_log_odds', 'log_odds_limit', 'min_evidence',
            'free_confirmations', 'save_median_kernel')}), flush=True)
    report = reconstruct(args.session.resolve(), output.resolve(), name,
                         temporal_filter=True, config_overrides=overrides)
    report['parameter_profile'] = str(args.mapping_config.resolve())
    report['parameter_profile_contents'] = args.mapping_config.read_text(encoding='utf-8')
    report['experiment_name'] = case
    report['explicit_parameter_overrides'] = explicit
    path = output / (name + '_report.json')
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    (output / 'parameters.yaml').write_text(yaml.safe_dump(
        {'semantic_mapper': report['effective_mapper_config']}, sort_keys=False), encoding='utf-8')


if __name__ == '__main__':
    main()
