OUTPUT_DIR="data/stp"
CMD="./src/bin/release/direction -d stp -h md -i 0-100 -a BAE-a BAE-p BAE-bfd-a TLBAE-a TLBAE-p DBBS-a DBBS-p"

mkdir -p $OUTPUT_DIR

echo "Running STP"
$CMD > "$OUTPUT_DIR/stp.out"
