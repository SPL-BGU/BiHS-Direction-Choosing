OUTPUT_DIR="data/pancake"
CMD="./src/bin/release/direction -d pancake -h 1 -i 0-100 -a BAE-a BAE-p BAE-bfd-a TLBAE-a TLBAE-p DBBS-a DBBS-p"

gaps=(0 1 2)

mkdir -p $OUTPUT_DIR

for gap in "${gaps[@]}"; do
  echo "Running Pancake with GAP-${gap}"
  $CMD -h "${gap}" > "$OUTPUT_DIR/pancake_${gap}.out"
done
