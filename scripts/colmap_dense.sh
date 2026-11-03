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
mkdir -p $OUT
$C image_undistorter --image_path $R/run_20261102_170423 --input_path $R/reconstruction/colmap_fixed_poses \
    --output_path $OUT/mvs --output_type COLMAP
$C patch_match_stereo --workspace_path $OUT/mvs --PatchMatchStereo.geom_consistency 1
$C stereo_fusion --workspace_path $OUT/mvs --output_path $OUT/fused.ply
$C poisson_mesher --input_path $OUT/fused.ply --output_path $OUT/meshed-poisson.ply
$C delaunay_mesher --input_path $OUT/mvs --output_path $OUT/meshed-delaunay.ply
rm -rf $OUT/mvs
ls -la $OUT
