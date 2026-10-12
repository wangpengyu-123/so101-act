#!/usr/bin/env bash
# =============================================================================
# SO-ARM101 ACT 5-minute smoke: train ~2 epochs -> save checkpoint -> (verify below)
#
# Usage:
#   ./act_smoke.sh <DATASET_REPO_ID_OR_ABS_PATH>
#
# e.g. ./act_smoke.sh $HOME/so101_walloss/so101_real_round2_rgb
#
# NOTE: this script sets HF_HUB_OFFLINE=1. Under offline mode, a hub-style id
# (e.g. "<user>/so101_YYYYMMDD_HHMMSS") only works if it is already cached under
# ~/.cache/huggingface/lerobot/<id>/. Otherwise pass an ABSOLUTE LOCAL PATH
# (e.g. $HOME/so101_walloss/so101_real_round2_rgb).
# The 2-epoch step count is computed from the dataset's real frame count, so it
# works for any dataset size. chunk_size / n_action_steps stay at the ACT default
# 100 (matching act_config.yml); do NOT shorten chunk for small datasets.
# =============================================================================
set -euo pipefail

DATASET_REPO_ID="${1:?usage: ./act_smoke.sh <DATASET_REPO_ID>}"
BATCH=8

source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
cd $HOME/wallx-workspace/wall-x/lerobot
export HF_HUB_OFFLINE=1

OUTPUT_DIR="outputs/train/act_smoke_$(date +%Y%m%d_%H%M%S)"

# 2 epochs = 2 * ceil(num_frames / batch_size). Metadata-only load (no video decode).
NUM_FRAMES=$(python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
print(LeRobotDatasetMetadata('${DATASET_REPO_ID}').total_frames)
")
STEPS=$(( (NUM_FRAMES * 2 + BATCH - 1) / BATCH ))
# Save ~once per epoch (2 checkpoints over the smoke run).
SAVE_FREQ=$(( (STEPS + 1) / 2 ))

echo "==> dataset frames = ${NUM_FRAMES}"
echo "==> 2-epoch steps  = ${STEPS} (batch ${BATCH})"
echo "==> save_freq      = ${SAVE_FREQ}"
echo "==> output_dir     = ${OUTPUT_DIR}"

lerobot-train \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.video_backend=pyav \
  --policy.type=act \
  --policy.device=cuda \
  --policy.use_vae=true \
  --policy.n_obs_steps=1 \
  --policy.chunk_size=100 \
  --policy.n_action_steps=100 \
  --policy.optimizer_lr=1e-5 \
  --policy.push_to_hub=false \
  --batch_size="${BATCH}" \
  --steps="${STEPS}" \
  --save_freq="${SAVE_FREQ}" \
  --num_workers=4 \
  --wandb.enable=false \
  --output_dir="${OUTPUT_DIR}" \
  --job_name="act_smoke"

echo ""
echo "SMOKE TRAIN DONE. Verify load+forward with:"
echo "  python verify_checkpoint.py ${OUTPUT_DIR}/checkpoints/last/pretrained_model"
