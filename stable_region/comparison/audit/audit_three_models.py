#!/usr/bin/env python3
"""Audit the saved regions in this checkout (requires NumPy and SciPy).

Inputs resolve relative to this script, so the checkout can be relocated.
By default only independent_audit.json in this audit folder is written.
Use --output-dir to keep an archived audit result unchanged during a rerun.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat

AUDIT_DIR = Path(__file__).resolve().parent
COMPARISON_DIR = AUDIT_DIR.parent
REPO_ROOT = COMPARISON_DIR.parent.parent
SOURCES = {
    'carsim': COMPARISON_DIR / 'reference_core',
    'carsim_fullcar': COMPARISON_DIR / 'carsim_fullcar',
    'simul_fullcar': COMPARISON_DIR / 'reference_core',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(model, key, angle):
    name = f'CarSim_region_{key}_core.mat' if model == 'carsim' else f'{model}_region_{key}_core.mat'
    path = SOURCES[model] / name
    d = loadmat(path, simplify_cells=True)['RESEARCH2D']
    c, meta = d['CASE'], d['meta']
    mu = meta.get('mu', meta.get('mu_road', meta.get('mu_road_manually_set')))
    wear = meta.get('wear', meta.get('wear_manually_set', c.get('wear')))
    assert np.array_equal(np.asarray(wear), [25, 25, 0, 0]), (path, 'wear')
    assert abs(float(mu) - .6) < 1e-14, (path, 'mu')
    assert float(meta['speed_kmh']) == 70, (path, 'speed')
    assert float(meta['steer_target_deg']) == angle, (path, 'steer')
    v, r = np.asarray(c['grid']['vc']), np.asarray(c['grid']['r'])
    assert v.shape == (61,) and r.shape == (121,), (path, 'axes')
    assert np.all(np.diff(v) > 0) and np.all(np.diff(r) > 0)
    assert np.allclose(np.diff(v), .1, rtol=0, atol=2e-15)
    assert np.allclose(np.diff(r), .025, rtol=0, atol=2e-15)
    cell = np.asarray(c['region']['cell_mask'], dtype=bool)
    point = np.asarray(c['masks']['final'], dtype=bool)[:, :, -1]
    h = np.asarray(c['handling']['mask'], dtype=bool)
    assert cell.shape == (60, 120) and point.shape == h.shape == (61, 121)
    assert np.all(~point | h), (path, 'final outside H')
    four = point[:-1, :-1] & point[1:, :-1] & point[:-1, 1:] & point[1:, 1:]
    assert np.all(~cell | four), (path, 'cell with rejected corner')
    weights = np.diff(v)[:, None] * np.diff(r)[None, :]
    area = float(weights[cell].sum())
    assert int(cell.sum()) == int(c['region']['accepted_cells'])
    assert abs(area - float(c['region']['cell_area'])) < 1e-12
    return dict(path=str(path), sha256=sha(path), meta=meta, v=v, r=r,
                cell=cell, point=point, h=h, weights=weights, area=area)


def pair(a, b):
    assert np.allclose(a['v'], b['v'], rtol=0, atol=1e-14)
    assert np.allclose(a['r'], b['r'], rtol=0, atol=1e-14)
    x, y = a['cell'], b['cell']
    common = a['h'] & b['h']
    different = np.argwhere(x != y)
    intersection, union = int((x & y).sum()), int((x | y).sum())
    difference_bounds = []
    for iv, ir in different:
        difference_bounds.append(dict(
            index_0based=[int(iv), int(ir)],
            vy_bounds_mps=[float(a['v'][iv]), float(a['v'][iv+1])],
            r_bounds_radps=[float(a['r'][ir]), float(a['r'][ir+1])],
            accepted_a=bool(x[iv, ir]), accepted_b=bool(y[iv, ir])))
    return dict(intersection_cells=intersection, union_cells=union,
                IoU=intersection/union, dice=2*intersection/(int(x.sum())+int(y.sum())),
                different_cells=int((x != y).sum()),
                only_a=int((x & ~y).sum()), only_b=int((y & ~x).sum()),
                weighted_IoU=float(a['weights'][x & y].sum()/a['weights'][x | y].sum()),
                common_H_points=int(common.sum()),
                common_H_point_agreement_count=int(((a['point'] == b['point']) & common).sum()),
                common_H_point_agreement=float((a['point'][common] == b['point'][common]).mean()),
                H_different_nodes=int((a['h'] != b['h']).sum()),
                max_vy_axis_difference=float(np.max(np.abs(a['v']-b['v']))),
                max_r_axis_difference=float(np.max(np.abs(a['r']-b['r']))),
                differing_cell_bounds=difference_bounds)


def sources_unchanged():
    root = REPO_ROOT
    sig = loadmat(root/'result/run_configuration.mat', simplify_cells=True)['Signature']
    expected = {'HMC_FullCar_Revised.m': sig['model_sha256'], 'data/hmc_data.mat': sig['data_sha256']}
    for dep in sig['dependencies']:
        if dep['name'] == 'available_hmc_mex_binary':
            continue
        name = 'calibration_active.json' if dep['name'] == 'active_calibration_file' else dep['name']
        expected[name] = dep['sha256']
    rows = []
    for name, digest in expected.items():
        actual = sha(root/name)
        rows.append(dict(path=name, expected=digest, actual=actual, match=actual == digest))
    assert all(row['match'] for row in rows)
    binding = json.loads((root/'stable_region/physics_binding.json').read_text())
    for name, digest in binding['physics_sha256'].items():
        assert sha(root/name) == digest
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=AUDIT_DIR,
                        help='JSON output folder (default: this audit folder)')
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = dict(source_signature_checks=sources_unchanged(), conditions={}, missing=[],
                  comparison_scope='Exact stored 10 s cell masks; no shifting, hull, interpolation, or retuning.',
                  caveats=[
                      'Finite-time sampled recovery and reference-H preservation are not an invariant-region certificate.',
                      'carsim and fresh carsim_fullcar use the same external H reconstructed at the recorded CarSim actual steering angle; simul_fullcar H matches the nominal commanded angle.',
                      'CarSim uses seven observed return channels. The LUT describes a 32-state full-car model but does not preserve its complete terminal tolerances or raw trajectories; a strict same-criterion three-way dynamics ranking is not established.',
                      'Original CarSim achieved initial-condition frame is not recorded; comparison uses corresponding nominal commanded initial-grid coordinates.',
                      'Points outside H are untested; point agreement is evaluated only in the intersection of tested H masks.'
                  ])
    for key, angle in [('0', 0.), ('1p5', 1.5), ('2p5', 2.5)]:
        try:
            models = {m: read(m, key, angle) for m in SOURCES}
        except FileNotFoundError as exc:
            result['missing'].append(str(exc.filename)); continue
        result['conditions'][key] = dict(
            speed_kmh=70, mu=.6, wear=[25,25,0,0], steer_target_deg=angle,
            models={m: dict(path=d['path'], sha256=d['sha256'],
                            accepted_cells=int(d['cell'].sum()), area=d['area'],
                            H_nodes=int(d['h'].sum()), final10_nodes=int(d['point'].sum()))
                    for m, d in models.items()},
            pairs={f'{a}__{b}': pair(models[a], models[b]) for a,b in itertools.combinations(models, 2)})
    result['complete'] = not result['missing']
    (output_dir/'independent_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    for key, condition in result['conditions'].items():
        print(key, {m: (d['accepted_cells'], d['area']) for m,d in condition['models'].items()})
        for names, p in condition['pairs'].items():
            print(' ', names, 'IoU=',p['IoU'],'different=',p['different_cells'], 'pointAgree=',p['common_H_point_agreement'])
    print('complete=', result['complete'], 'source signatures=', len(result['source_signature_checks']))

if __name__ == '__main__':
    main()
