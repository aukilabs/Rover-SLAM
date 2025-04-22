INPUT_ROOT_DIR="$1"
OUTPUT_ROOT_DIR="$2"
SCAN_SUBFOLDER="$3"

SCAN_FOLDERS=()

if [ "$SCAN_SUBFOLDER" = "--all" ]; then
    for SCAN_FOLDER in "$INPUT_ROOT_DIR"/*; do
        SCAN_FOLDERS+=("$SCAN_FOLDER")
    done
else
    SCAN_FOLDERS=("$SCAN_FOLDER")
fi

for SCAN_FOLDER in "${SCAN_FOLDERS[@]}"; do
    SCAN_ID=$(basename "$SCAN_FOLDER")
    OUT_FOLDER="$OUTPUT_ROOT_DIR/$SCAN_ID"

    echo "Running SLAM on $SCAN_FOLDER"
    mkdir -p "$OUT_FOLDER"

    ./Examples/Monocular-Inertial/mono_inertial_dmt \
    Vocabulary/voc_binary_tartan_8u_6.yml.gz \
    ./Examples/Monocular-Inertial/DMT.yaml \
    $SCAN_FOLDER \
    $OUT_FOLDER
done
