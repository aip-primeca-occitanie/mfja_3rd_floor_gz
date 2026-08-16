# Room 315 visual-state V4 dataset card

## Summary

Synthetic Gazebo overhead RGB data used for the Room 315 split-rail visual
state V4 experiment. This complete post-experiment distribution contains
7,336 paired-camera scenes and 14,672 images: 5,528 Training, 512
Validation, 256 Development-Canary, and 1,040 Final-Test scenes.

Every scene has exactly two 640x480 JPEG images (`left_rail_rgb` and
`right_rail_rgb`) and one oracle visual-state label containing the fixed
eight-shuttle/eight-switch state contract.

## Intended use

Reproduce or audit the Room 315 V4 visual-state experiment in simulation.
The data does not establish physical robot safety or deployment approval.

## Partition policy

- Training: included, with two sources kept separate for 0.5/0.5 sampling.
- Validation: included as a separate release asset.
- Canary: included and clearly marked as post-selection development
  regression; it was not used for checkpoint selection.
- Final Test: included after the immutable one-shot attempt completed; it
  was never used for gradient updates or model selection.
- Historical old-replay validation/test: not included.

## Privacy and sanitization

Captures are synthetic and contain no people. Raw capture logs and capture
state are excluded. Four absolute trace-path fields were removed from each
operational old-replay row; image inputs and labels are unchanged. Some
byte-exact historical evidence JSON intentionally retains obsolete local
paths so its published SHA-256 remains auditable; path-sanitized reading
copies are supplied beside it. The original old-replay rows and their
selected event/validation trace files are retained under provenance and are
never used by the portable operational loader.

## Model evidence

The model asset contains both the V3 initialization checkpoint and the
epoch-11 V4 checkpoint selected on Validation
(`869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d`), plus the complete frozen candidate and
result evidence. Canary passed and made the candidate eligible for manual
runtime review; Final Test passed; neither result automatically activated a
runtime model.

## Reproduction scope

The bundle supports input verification, repetition of the reported training
procedure, and stateless re-evaluation of the published checkpoint on all
three labelled evaluation partitions. It does not claim that retraining on
different hardware will reproduce the checkpoint file bit-for-bit.

## License

See `LICENSE_STATUS.md`. Public access is not described as an open-data
license grant.
