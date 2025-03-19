import sys
from collections import defaultdict


def parse_results(filename):
    total_solution = defaultdict(float)
    total_expanded = defaultdict(int)
    total_fabove = defaultdict(int)
    total_time = defaultdict(float)
    count = defaultdict(int)

    with open(filename, 'r') as file:
        for line in file:
            if line.startswith("[R]"):
                parts = line.split('; ')
                alg = parts[0].split('alg: ')[1]
                solution = float(parts[1].split('solution ')[1])
                expanded = int(parts[2].split('expanded: ')[1])
                fabove = int(parts[3].split('fabove: ')[1])
                time = float(parts[4].split('time: ')[1].strip('s'))

                total_solution[alg] += solution
                total_expanded[alg] += expanded
                total_fabove[alg] += fabove
                total_time[alg] += time
                count[alg] += 1

    return total_solution, total_expanded, total_fabove, total_time, count


def summarize_results(filename):
    total_solution, total_expanded, total_fabove, total_time, count = parse_results(filename)

    print("Algorithm Summary:")
    for alg in count:
        avg_solution = total_solution[alg] / count[alg]
        avg_expanded = total_expanded[alg] / count[alg]
        avg_fabove = total_fabove[alg] / count[alg]
        avg_time = total_time[alg] / count[alg]

        print(f"Algorithm: {alg}")
        print(f"  Avg Solution Cost: {avg_solution:.2f}")
        print(f"  Avg Nodes Expanded: {avg_expanded:.2f}")
        print(f"  Avg FAbove: {avg_fabove:.2f}")
        print(f"  Avg Time: {avg_time:.6f} s\n")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python summarize_results.py <results_file>")
        sys.exit(1)
    summarize_results(sys.argv[1])
