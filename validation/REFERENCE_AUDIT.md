# CarSim reference extraction audit

All three references were decoded to MATLAB v7 compatible flat files. Each contains 10,001 samples and all 72 original float64 EXPORT channels. Raw sample bytes, reconstructed timestamps, and every original `info` field survived MAT save/reload checks. The original source files remain unchanged.

| Flat file | Embedded speed (km/h) | Embedded mu | Embedded steer command (deg) | Recorded time (s) |
| --- | ---: | ---: | ---: | --- |
| `flat/CarSim_50.mat` | 50 | 0.5 | 0 | 12–22 |
| `flat/CarSim_70.mat` | 70 | 0.6 | 2.5 | 12–22 |
| `flat/CarSim_90.mat` | 90 | 0.6 | 2.5 | 12–22 |

The 50 km/h source filename incorrectly suggests mu 0.6 and steer 2.5. Its original embedded metadata records mu 0.5 and steer 0, matching the supplied metrics CSV. Conditions were copied from original metadata, never inferred from filenames.

The source timeseries stores regular time through MCOS `Start_=12`, `Increment_=0.001`, `Length=10001`, with an empty explicit `Time_` array. The extractor decodes `Start_ + Increment_ * arange(Length)`, the same arithmetic used by `tsdata.timemetadata.getData`. This restores the recorded time representation; it performs no fitted shift or resampling. `read_carsim_reference` then subtracts the first recorded timestamp to yield 0–10 s.

Data extraction only transposes HDF5 dimension order from 72×10001 to 10001×72. It preserves all channels and source float64 samples. No amplitude correction, smoothing, transient cut, or channel rearrangement is applied. The existing reader retains its existing EXPORT channel schema; this task verifies preservation, not an independent reconstruction of exporter labels.

Octave successfully loaded all three flat files through the existing `read_carsim_reference`. Checks confirmed complete condition metadata, exact raw array preservation, 0–10 s relative time, and the reader's documented velocity/rate conversions.

`reference_manifest.json` records input and flat-file SHA-256 values, raw array byte hashes, version information, and matching conditions. Individual reports preserve all original metadata. `flatten_carsim_reference.py` accepts an input MAT path, `--output`, and optional `--report-json`. Install the three packages in `requirements.txt` to run it.

These checks only cover extraction and reference reading. They do not assert that model metrics reproduce the expected CSV.
