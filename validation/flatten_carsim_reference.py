#!/usr/bin/env python3
"""Flatten a CarSim MATLAB v7.3 timeseries without altering raw samples.

Usage:
  PYTHONPATH=/path/to/h5py python3 flatten_carsim_reference.py INPUT.mat \
      --output FLAT.mat [--report-json AUDIT.json]

This tool supports the MCOS v4 property-reference encoding found in the
attached CarSim timeseries files. It reads numeric payloads and metadata;
it never executes serialized MATLAB objects or reconstructs conditions
from filenames. Uniform recorded time is decoded from Start_, Increment_,
and Length using the same arithmetic as tsdata.timemetadata.getData.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.io import loadmat, savemat


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def array_sha256(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def matlab_class(obj):
    value = obj.attrs.get('MATLAB_class', b'')
    return value.decode() if isinstance(value, bytes) else str(value)


def decode_metadata(obj, handle):
    """Decode plain MATLAB info groups/cells without executing MCOS objects."""
    if isinstance(obj, h5py.Group):
        return {name: decode_metadata(child, handle) for name, child in obj.items()}
    raw = obj[()]
    if obj.attrs.get('MATLAB_empty', 0):
        shape = tuple(int(x) for x in np.asarray(raw).reshape(-1)[::-1])
        return np.empty(shape, dtype=float)
    view = raw.transpose(tuple(reversed(range(raw.ndim))))
    if matlab_class(obj) == 'char':
        if view.ndim > 2 or (view.ndim == 2 and view.shape[0] != 1):
            raise ValueError(f'Unsupported metadata char matrix: {obj.name}')
        return np.asarray(view, dtype='<u2').tobytes().decode('utf-16le')
    if raw.dtype == h5py.ref_dtype:
        decoded = np.empty(view.shape, dtype=object)
        for index in np.ndindex(view.shape):
            reference = view[index]
            decoded[index] = decode_metadata(handle[reference], handle) if reference else ''
        return decoded
    if matlab_class(obj) == 'logical':
        return view.astype(bool)
    if raw.dtype.kind not in 'biufc':
        raise ValueError(f'Unsupported metadata type {raw.dtype}: {obj.name}')
    return view


def property_datasets(handle):
    """Find unique named MCOS external property payloads for this file family."""
    wrapper = handle['#subsystem#/MCOS']
    references = wrapper[()].reshape(-1)
    blob = handle[references[0]][()].tobytes()
    if len(blob) < 40:
        raise ValueError('MCOS metadata header is incomplete')
    header = np.frombuffer(blob[:40], dtype='<u4')
    if int(header[0]) != 4:
        raise ValueError(f'Unsupported MCOS version: {header[0]}')
    string_end, properties_begin, properties_end = map(int, (header[2], header[5], header[6]))
    if not 40 <= string_end <= properties_begin <= properties_end <= len(blob):
        raise ValueError('MCOS section offsets are invalid')
    names = [x.decode('utf-8') for x in blob[40:string_end].split(b'\0') if x]
    if len(names) != int(header[1]):
        raise ValueError('MCOS property-string count differs from its header')
    properties = np.frombuffer(blob[properties_begin:properties_end], dtype='<u4')
    result = {}
    for key in ('Data_', 'Length', 'Time_', 'Increment_', 'Start_'):
        property_id = names.index(key) + 1
        candidates = []
        for i in range(len(properties) - 2):
            # Storage kind 1 is an external payload. The first two wrapper
            # cells hold MCOS metadata and its canonical-empty value.
            ref_index = int(properties[i + 2]) + 2
            if properties[i] == property_id and properties[i + 1] == 1 and ref_index < len(references):
                candidate = handle[references[ref_index]]
                if isinstance(candidate, h5py.Dataset):
                    candidates.append(candidate.name)
        candidates = sorted(set(candidates))
        if len(candidates) != 1:
            raise ValueError(f'Expected one MCOS payload for {key}, got {candidates}')
        result[key] = handle[candidates[0]]
    return result


def scalar_dataset(dataset):
    raw = dataset[()]
    if raw.size != 1 or raw.dtype.kind not in 'iuf' or not np.isfinite(raw).all():
        raise ValueError(f'Expected one finite numeric time parameter: {dataset.name}')
    return float(raw.item())


def json_value(value):
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, list):
        return [json_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def check_metadata_preserved(expected, actual, prefix='info'):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            raise AssertionError(f'Metadata fields changed at {prefix}')
        for key in expected:
            check_metadata_preserved(expected[key], actual[key], f'{prefix}.{key}')
    else:
        np.testing.assert_array_equal(np.asarray(expected).squeeze(), np.asarray(actual).squeeze(),
                                      err_msg=f'Metadata changed at {prefix}')


def flatten(source, destination, report_path=None):
    source = Path(source).resolve()
    destination = Path(destination).resolve()
    if source == destination:
        raise ValueError('Output must differ from the immutable original input')
    source_sha = sha256_file(source)
    with h5py.File(source, 'r') as handle:
        if matlab_class(handle['CarSimOut_trim']) != 'timeseries':
            raise ValueError('Input does not declare CarSimOut_trim as a timeseries')
        payload = property_datasets(handle)
        raw_source = payload['Data_'][()]
        if raw_source.dtype != np.dtype('float64') or raw_source.ndim != 2:
            raise ValueError('Expected a 2-D raw float64 export payload')
        data = raw_source.T.copy()
        if data.shape[1] != 72 or not np.isfinite(data).all():
            raise ValueError(f'Expected finite N-by-72 raw export data, got {data.shape}')
        encoded_time = payload['Time_']
        if encoded_time.attrs.get('MATLAB_empty', 0):
            count_value = scalar_dataset(payload['Length'])
            if not count_value.is_integer() or count_value < 2:
                raise ValueError('Recorded sample Length is not an integer >= 2')
            count = int(count_value)
            start = scalar_dataset(payload['Start_'])
            increment = scalar_dataset(payload['Increment_'])
            if increment <= 0:
                raise ValueError('Recorded uniform time Increment_ must be positive')
            time = start + increment * np.arange(count, dtype=np.float64)
            time_encoding = 'recorded_uniform_Start_Increment_Length'
            time_operation = 'Decode recorded Start_ + Increment_ * arange(Length); no fitted shift or resampling'
        else:
            time = np.asarray(encoded_time[()], dtype=float).reshape(-1)
            count, start = time.size, float(time[0])
            increment = float('nan')
            time_encoding = 'recorded_explicit_Time_array'
            time_operation = 'Copy recorded numeric Time_ vector; no fitted shift or resampling'
        if count != data.shape[0] or not np.isfinite(time).all() or not np.all(np.diff(time) > 0):
            raise ValueError('Time does not match the data samples or is not strictly increasing')
        info = decode_metadata(handle['info'], handle) if 'info' in handle else {}
        for field, observed in [('start_s', time[0]), ('stop_s', time[-1])]:
            if field in info and abs(float(np.asarray(info[field]).item()) - observed) > 1e-10:
                raise ValueError(f'Embedded info.{field} differs from decoded time')
        conditions = {}
        for key in ('speed_kmh', 'mu', 'steer_command_deg'):
            conditions[key] = float(np.asarray(info[key]).item()) if key in info else 'unknown'
        complete = all(value != 'unknown' for value in conditions.values())
        provenance = dict(
            extractor='flatten_carsim_reference.py', source_file=str(source),
            source_sha256=source_sha, source_hdf5_data_dataset=payload['Data_'].name,
            source_hdf5_shape=np.asarray(raw_source.shape, dtype=np.int64),
            raw_source_array_sha256=array_sha256(raw_source),
            normalized_raw_array_sha256=array_sha256(data),
            data_operation='Transpose HDF5 dimension order to N-by-72 only; all original float64 samples preserved',
            time_encoding=time_encoding, time_operation=time_operation,
            recorded_length=count, recorded_start_s=start,
            recorded_increment_s=(increment if np.isfinite(increment) else 'not_applicable_explicit_time'),
            time_property_datasets=json.dumps({key: value.name for key, value in payload.items() if key != 'Data_'}),
            amplitude_operation='None', samples_removed=0,
            condition_metadata_status='embedded_complete' if complete else 'unknown_or_incomplete',
            filename_conditions_used=False,
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    savemat(destination, {'ref': dict(raw_export_data=data, time_absolute_s=time[:, None],
                                    info=info, provenance=provenance)},
            do_compression=True, long_field_names=True, oned_as='column')
    restored = loadmat(destination, simplify_cells=True)['ref']
    np.testing.assert_array_equal(restored['raw_export_data'], data)
    np.testing.assert_array_equal(restored['time_absolute_s'], time)
    if array_sha256(restored['raw_export_data'].T.copy()) != array_sha256(raw_source):
        raise AssertionError('Round-trip raw array byte hash differs from source payload')
    check_metadata_preserved(info, restored['info'])
    if sha256_file(source) != source_sha:
        raise AssertionError('Source input changed during conversion')
    report = dict(
        source_file=str(source), source_sha256=source_sha,
        flat_file=str(destination), flat_sha256=sha256_file(destination),
        raw_hdf5_shape=list(raw_source.shape), raw_flat_shape=list(data.shape),
        source_dtype=str(raw_source.dtype), all_samples_finite=True,
        sample_count=count, channel_count=72,
        time_start_s=float(time[0]), time_stop_s=float(time[-1]),
        time_encoding=time_encoding, recorded_increment_s=(increment if np.isfinite(increment) else None),
        observed_time_step_min_s=float(np.diff(time).min()),
        observed_time_step_max_s=float(np.diff(time).max()),
        raw_source_array_sha256=array_sha256(raw_source),
        normalized_raw_array_sha256=array_sha256(data),
        raw_round_trip_byte_hash_identical=True, time_round_trip_identical=True,
        all_info_fields_preserved=True, embedded_conditions=conditions,
        condition_metadata_status=provenance['condition_metadata_status'],
        first_fourteen_raw_channels=data[0, :14].tolist(),
        final_fourteen_raw_channels=data[-1, :14].tolist(),
        info=json_value(info), provenance=json_value(provenance),
    )
    if report_path:
        Path(report_path).parent.mkdir(parents=True, exist_ok=True)
        Path(report_path).write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report-json', type=Path)
    args = parser.parse_args()
    report = flatten(args.input, args.output, args.report_json)
    print(json.dumps({key: report[key] for key in ('flat_file', 'source_sha256', 'raw_flat_shape',
        'time_start_s', 'time_stop_s', 'embedded_conditions', 'raw_round_trip_byte_hash_identical')}, indent=2))


if __name__ == '__main__':
    main()
