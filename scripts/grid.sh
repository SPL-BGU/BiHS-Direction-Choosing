OUTPUT_DIR="data/grid"
MAP_DIR="maps"
SCEN_DIR="scenarios"
CMD="./src/bin/release/direction -d grid -h od -i 0-10000 -a BAE-a BAE-p BAE-bfd-a TLBAE-a TLBAE-p DBBS-a DBBS-p"

mkdir -p $OUTPUT_DIR

for file in ${MAP_DIR}/*.map; do
    filename=$(basename "$file" .map)
    echo "Running Grid - ${filename}"
    $CMD -m "${MAP_DIR}/${filename}.map" -s "${SCEN_DIR}/${filename}.map.scen" > "$OUTPUT_DIR/grid_${filename}.out"
done
