#!/bin/bash
# COLMAP dense reconstruction of exploration run 3 with the robot's camera poses, on one GPU.
# From the repo root:  sbatch -A <account> scripts/colmap_dense.sh
# The sparse model (colmap_fixed_poses) comes from scripts/colmap_known_poses.py + point_triangulator.
#SBATCH --job-name=colmap_dense
#SBATCH --partition=savio3_gpu
#SBATCH --qos=savio_lowprio
#SBATCH --gres=gpu:GTX2080TI:1
#SBATCH --cpus-per-task=2
#SBATCH --time=01:00:00
#SBATCH --output=colmap_dense_%j.log
set -e
cd "$SLURM_SUBMIT_DIR"
R=results/explore_2026-11-02c
OUT=$R/reconstruction/dense
command -v apptainer >/dev/null || module load apptainer 2>/dev/null || true
RUN=$(command -v apptainer || command -v singularity)
SIF=${SIF:-/global/scratch/users/$USER/colmap.sif}
export APPTAINER_CACHEDIR=/global/scratch/users/$USER/.apptainer
[ -f "$SIF" ] || $RUN pull "$SIF" docker://colmap/colmap:latest
C="$RUN exec --nv $SIF colmap"
nvidia-smi
rm -rf $OUT && mkdir -p $OUT
$C image_undistorter --image_path $R/run_20261102_170423 --input_path $R/reconstruction/colmap_fixed_poses     --output_path $OUT/mvs --output_type COLMAP
# Source images picked from the robot poses (scripts/colmap_patchmatch_cfg.py), not from SIFT tracks.
cp $R/reconstruction/patch-match.cfg $OUT/mvs/stereo/patch-match.cfg
$C patch_match_stereo --workspace_path $OUT/mvs --PatchMatchStereo.geom_consistency 1     --PatchMatchStereo.depth_min 0.15 --PatchMatchStereo.depth_max 8     --PatchMatchStereo.filter_min_triangulation_angle 1.5
FUSE="--StereoFusion.min_num_pixels 3 --StereoFusion.max_reproj_error 4 --StereoFusion.max_depth_error 0.02"
$C stereo_fusion --workspace_path $OUT/mvs --input_type photometric --output_path $OUT/fused_photometric.ply $FUSE
$C stereo_fusion --workspace_path $OUT/mvs --input_type geometric --output_path $OUT/mvs/fused.ply $FUSE
cp $OUT/mvs/fused.ply $OUT/fused.ply
$C poisson_mesher --input_path $OUT/fused.ply --output_path $OUT/meshed-poisson.ply || true
$C delaunay_mesher --input_path $OUT/mvs --output_path $OUT/meshed-delaunay.ply || true
ls -la $OUT
