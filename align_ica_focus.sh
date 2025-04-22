DATASETS_ROOT_DIR="$1"
OUTPUTS_ROOT_DIR="$2"
SCAN_SUBFOLDER="$3"
TRUTH_PORTALS_JSON="$4"

SCAN_FOLDERS=()

if [ "$SCAN_SUBFOLDER" = "--all" ]; then
    for SCAN_FOLDER in "$OUTPUTS_ROOT_DIR"/*; do
        SCAN_FOLDERS+=("$SCAN_FOLDER")
    done
else
    SCAN_FOLDERS=("$SCAN_FOLDER")
fi


TRUTH_ARG=""
if [ -n "$TRUTH_PORTALS_JSON" ]; then
    TRUTH_ARG="--truth $TRUTH_PORTALS_JSON"
fi

for SCAN_FOLDER in "${SCAN_FOLDERS[@]}"; do
    SCAN_ID=$(basename "$SCAN_FOLDER")
    python3 ../reconstruction-server/dev-utils/align_cam_traj.py \
        "$OUTPUTS_ROOT_DIR/$SCAN_ID/CameraTrajectory.txt" \
        "$DATASETS_ROOT_DIR/$SCAN_ID/ARposes.csv" \
        --out "$OUTPUTS_ROOT_DIR/$SCAN_ID/AlignedTraj.csv" \
        --plot
    python3 ../reconstruction-server/dev-utils/plot_qr_and_traj.py \
        "$DATASETS_ROOT_DIR/$SCAN_ID/" \
        --refined-traj "$OUTPUTS_ROOT_DIR/$SCAN_ID/AlignedTraj.csv" \
        $TRUTH_ARG
done
