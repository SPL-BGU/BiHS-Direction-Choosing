from decimal import Decimal, ROUND_HALF_UP
from os import PathLike
from pathlib import Path

import pandas as pd
from pandas import DataFrame


def round_half_up(value, decimals):
    d = Decimal(value)
    return d.quantize(Decimal('1e-{0}'.format(decimals)), rounding=ROUND_HALF_UP)


def parse_file(file_path: Path) -> DataFrame:
    current_dict = {}
    dicts = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('[D]') or line.startswith('[I]') or line.startswith('[R]'):
                current_dict.update(dict(item.split(": ") for item in line[3:].strip().split("; ")))
                if line.startswith('[R]'):
                    dicts.append(current_dict.copy())
    return DataFrame(dicts)


def parse_dir(dir_path: str | PathLike[str]):
    dir_path = Path(dir_path)
    dfs = [parse_file(file_path) for file_path in dir_path.rglob('*') if file_path.suffix in ['.out', '.txt', '.log']]
    df = pd.concat(dfs, ignore_index=True)

    # Convert types
    df['id'] = df['id'].astype(int)
    df['solution'] = df['solution'].astype(float)
    df['expanded'] = df['expanded'].astype(int)
    df['fabove'] = df['fabove'].fillna(0)
    df['fabove'] = df['fabove'].astype(int)
    df['time'] = df['time'].str.replace('s', '', regex=False).astype(float)

    return df


def verify_solutions(df):
    inconsistent = df.groupby(['id'])['solution'].nunique() > 1
    if inconsistent.any():
        raise ValueError(f"Inconsistent solutions found")


def verify_fabove(df):
    # List of algorithms to check
    algorithms_to_check = {"BAE-bfd-a", "BAE-bfd-f", "BAE-bfd-b", "TLBAE"}

    # Filter rows with the specified algorithms
    filtered_df = df[df['alg'].isin(algorithms_to_check)]

    # Check if any 'fabove' value is not zero
    if (filtered_df['fabove'] != 0).any():
        raise ValueError(f"'fabove' is not 0 for some cases")


def verify_count(df):
    if df['algorithm'].value_counts().nunique() != 1:
        raise ValueError(f"'Not all instances were run for all algorithms")


def write_to_excel(df, filename="results/pancake.xlsx"):
    with pd.ExcelWriter(filename) as writer:
        df.to_excel(writer, sheet_name="results", index=False)


def main():
    data_dir = r"data/pancake"
    Path("results").mkdir(exist_ok=True)
    print("Loading data")
    df = parse_dir(data_dir)
    print("Verifying Correctness")
    verify_solutions(df)
    verify_fabove(df)
    print("Generating Excel")
    write_to_excel(df)


if __name__ == '__main__':
    main()
