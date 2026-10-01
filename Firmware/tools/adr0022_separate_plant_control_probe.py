"""Freeze fitted controllers, then test a separate supplied synthetic plant."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, require
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.tools.adr0022_nuisance_verification import true_model
from Firmware.tools.adr0022_fresh_family_control_probe import load_fits, fitted_asset, runtime_receipt, control_case, save
from Firmware.tools.adr0022_fitted_start_policy_probe import policy_support, source_margin_records, compact_start_case
from Firmware.tools.adr0022_fitted_candidate_qualification_probe import mapped_gains, controller_parameters_for
from Firmware.tools.adr0022_start_policy_synthesis_probe import case_label

METHOD = 'adr0022.separate-synthetic-plant-control/1'
CASES = tuple({'kind': 'plateau', 'direction': direction, 'speed_deg_s': direction*5., 'noisy': noisy, 'seed': seed}
              for direction in (-1, 1)
              for noisy, seed in ((False, 767003), (True, 767003), (True, 767017)))
CURVE = (2., 1.5)

def declaration(source, margins, triples, locals_by_asset):
    return {'schema': METHOD, 'fitted_models': [a.model.document() for a, _, _ in triples],
        'fit_source': str(source.resolve()), 'margin_source': str(margins.resolve()),
        'model_revisions': [a.model_revision for a, _, _ in triples],
        'curve': list(CURVE), 'gains': [mapped_gains(a, local, CURVE).__dict__
            for (a, _, _), local in zip(triples, locals_by_asset)],
        'START_excess_A': {'negative': .020, 'positive': .020},
        'mandatory_cases_per_model': list(CASES), 'mandatory_cases': 18,
        'plant': true_model().document(),
        'plant_role': 'separate supplied synthetic truth; controller receives simulated sensors only',
        'plant_parameters_injected_into_controller': False,
        'gyro_source_clock': 'actual plant source delay; known common synthetic clock',
        'all_references_and_gates': 'unchanged complete signed5deg/s plateaux and fixed2s stop',
        'unknown_to_controller': 'ten supplied plant coordinates differ from each fitted controller model',
        'new_noise_draws': '767003/767017; same consumed reference design and supplied truth point',
        'noise_seed_lifetime_admission': 'NOT_CLAIMED',
        'uncertainty': 'UNKNOWN', 'whole_controller_qualified': False,
        'physical_stage3a': 'NOT_RUN', 'physical_stage3b': 'NOT_RUN', 'deployment_authorized': False}

def inputs(args):
    records = load_fits(args.fits)
    triples = [fitted_asset(args.fits, record) for record in records]
    locals_by_asset = source_margin_records(args.margins, triples)
    return triples, locals_by_asset

def require_case_coverage(revisions, rows):
    expected = [(revision, case) for revision in revisions for case in CASES]
    require([(row['model_revision'], row['case']) for row in rows] == expected,
        Reason.DATA_INVALID, 'exact ordered18 mandatory model/case dictionaries required')

def one(args, native, family, triple, local, case, folder, plant):
    asset, support, state = triple
    support = policy_support(support, .020)
    folder.mkdir(parents=True, exist_ok=False)
    parameters, support, _ = runtime_receipt(asset, support, native,
        controller_parameters_for(asset, mapped_gains(asset, local, CURVE)), folder)
    row = control_case(native, family, asset, support, state, parameters, case,
        folder/case_label(case), plant_model=plant)
    return parameters, row, folder/case_label(case)/'trace.npz'

def run(args):
    triples, locals_by_asset = inputs(args)
    protocol = declaration(args.fits, args.margins, triples, locals_by_asset)
    native, family = Native(args.library), FamilyNative(args.library)
    if args.stage == 'early':
        args.output.mkdir(parents=True, exist_ok=False)
        save(args.output/'predeclared-contract.json', protocol)
        baseline_case = {'kind': 'plateau', 'direction': -1, 'speed_deg_s': -5., 'noisy': False, 'seed': 101}
        _, baseline, newpath = one(args, native, family, triples[0], locals_by_asset[0],
            baseline_case, args.output/'default-replay', None)
        oldpath = args.baseline/'combined-pair'/triples[0][0].model_revision/case_label(baseline_case)/'trace.npz'
        with np.load(newpath, allow_pickle=False) as new, np.load(oldpath, allow_pickle=False) as old:
            arrays = {key: bool(np.array_equal(new[key], old[key])) for key in old.files}
        require(all(arrays.values()), Reason.INTEGRATION_MISMATCH,
            'optional separate-plant support must preserve actual retained default arrays exactly')
        save(args.output/'default-replay-decision.json', {'all_arrays_exact': all(arrays.values()),
            'arrays': arrays, 'case': baseline_case, 'new_source_acceptance_or_limits': False})
        _, row, _ = one(args, native, family, triples[0], locals_by_asset[0], CASES[0], args.output/'early', true_model())
        save(args.output/'early-result.json', row)
        return
    require(json.loads((args.output/'predeclared-contract.json').read_text()) == protocol,
        Reason.INTEGRATION_MISMATCH, 'models, own gains, references, new noise and guards remain frozen')
    cases, compact = [], []
    for index, (triple, local) in enumerate(zip(triples, locals_by_asset)):
        for case in CASES:
            if index == 0 and case == CASES[0]:
                row = json.loads((args.output/'early-result.json').read_text())
                support = policy_support(triple[1], .020)
                from Firmware.commissioning.family_assets import bind_diagnostic_runtime
                parameters, _, _ = bind_diagnostic_runtime(triple[0], native,
                    controller_parameters_for(triple[0], mapped_gains(triple[0], local, CURVE)), support)
            else:
                parameters, row, _ = one(args, native, family, triple, local, case,
                    args.output/'cases'/triple[0].model_revision/case_label(case), true_model())
            cases.append(row)
            compact.append(compact_start_case('separate-plant', .020, .020, triple[0], parameters, row))
            save(args.output/'partial-results.json', cases)
    require_case_coverage(tuple(triple[0].model_revision for triple in triples), cases)
    # This freeze declares its new noise cases explicitly and retains the
    # existing per-case motion predicates.
    from Firmware.tools.adr0022_start_policy_synthesis_probe import case_passed
    passed = all(case_passed(r) for r in cases)
    with (args.output/'case-results.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(compact[0]))
        writer.writeheader(); writer.writerows(compact)
    decision = {'schema': METHOD, 'status': 'SEPARATE_SYNTHETIC_PLANT_PLATEAUX_PASS' if passed else 'SEPARATE_PLANT_MOTION_FAILED',
        'cases': 18, 'completed': sum(r['completed'] for r in cases),
        'original_quality_passes': sum(r['original_quality_passed'] for r in cases),
        'owner_quality_passes': sum(r['revised_quality_passed'] for r in cases),
        'independent_forward_passes': sum(r['conditional_forward_passed'] for r in cases),
        'exact_native_readbacks': sum(r['actual_controller_readback_count'] for r in cases),
        'default_saved_array_replay': 'PASS', 'early_case_reused_once': True,
        'scope': 'fixed supplied synthetic truth point; same consumed reference design, new noise draws; not physical or global validation',
        'uncertainty': 'UNKNOWN', 'controller_qualified': False,
        'physical_stage3a': 'NOT_RUN', 'physical_stage3b': 'NOT_RUN', 'deployment_authorized': False}
    save(args.output/'decision.json', decision)
    print(json.dumps(decision), flush=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('fits', 'margins', 'library', 'baseline', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    parser.add_argument('--stage', required=True, choices=('early', 'full'))
    run(parser.parse_args())
