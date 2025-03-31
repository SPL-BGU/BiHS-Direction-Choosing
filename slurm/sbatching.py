import argparse
from pathlib import Path
import signal
import subprocess
from types import FrameType
from typing import Optional, Union, List


def sigint_handler(sig: int, frame: FrameType) -> None:
    """
    Handles SIGINT (Ctrl-C) signal, prompting the user to confirm quitting the program.

    Parameters:
        sig (int): The signal number.
        frame (FrameType): The current stack frame (unused).

    Note:
        This is not a very secure method, so recovery isn't guaranteed once pressed

    """
    signal.signal(signal.SIGINT, sigint_handler)
    print('\nCtrl-C pressed. Do you want to quit? (y/n): ', end="")
    response = input().strip().lower()
    if response in ['y', 'yes']:
        Path(sbatch_file_name).unlink()
        exit(0)


def progress_bar(progress, total) -> None:
    """
        Displays a progress bar in the console.

        Parameters:
            progress (int): The current progress value.
            total (int): The total value indicating completion.

        """
    percent = 100 * (progress / float(total))
    percent_to_show = int(percent) // 2
    bar = '❚' * percent_to_show + '-' * (50 - percent_to_show)
    print(f'\r|{bar}| {percent:.2f}%', end='\r')
    if progress == total:
        print()


def submit_job(run_lines: Optional[Union[List[str], str]], partition: str = 'main', job_name: str = 'job',
               runtime: str = '1-00:00:00', cpus_per_task: int = -1, mem: int = -1, suppress_output: bool = False,
               output_file: str = 'slurm-%j.out', suppress_error: bool = False, error_file: bool = None,
               dependency: Optional[str] = None, conda_env: Optional[str] = None, force_gpu: bool = False) -> int:
    """
    ""
    Submits a job to a SLURM workload manager with specified parameters.

    Parameters:
        run_lines (Union[List[str], str]): Commands to run in the job
        partition (str): The partition to run the job on. Default is 'main'.
        job_name (str): The name of the job. Default is 'job'.
        runtime (str): The job's runtime limit. Default is '1-00:00:00'.
        cpus_per_task (int): Number of CPUs per task. Default is 1.
        mem (int): Memory allocation in GB. Default is -1 (no memory specified).
        suppress_output (bool): If True, suppresses standard output. Default is False.
        output_file (str): File to write standard output. Default is 'slurm-%j.out'.
        suppress_error (bool): If True, suppresses error output. Default is False.
        error_file (bool): File to write error output. If one is not specified, it will be directed to the output_file
        dependency (Optional[str]): Job dependency.
        conda_env (Optional[str]): Conda environment to activate.
        force_gpu (bool): If True, ensures GPU is set when requested. Default is False.

    Note:
        Do not change partition unless you have a good reason to
        You can read more about these options here: https://slurm.schedmd.com/sbatch.html

    Returns:
        int: The job ID of the submitted job.

    Raises:
        Exception: If no commands are given to run, if GPU is not set when requested, or if an unknown error occurs.
    """
    # Write all the parameters into the temporary bash file
    with open(sbatch_file_name, 'w+', newline="\n") as f:
        f.write('#!/bin/bash\n')
        f.write(f"#SBATCH --partition {partition}\n")
        f.write(f'#SBATCH --time {runtime}\n')
        f.write(f'#SBATCH --constraint cpu256\n')
        f.write(f'#SBATCH --exclude ise-cpu256-01,ise-cpu256-06\n')
        if cpus_per_task > 0:
            f.write(f'#SBATCH --cpus-per-task={cpus_per_task}\n')
        f.write(f'#SBATCH --job-name {job_name}\n')

        f.write(f'#SBATCH --output {"/dev/null" if suppress_output else output_file}\n')

        if suppress_error:
            f.write('#SBATCH --error /dev/null\n')
        elif error_file:
            f.write(f'#SBATCH --error {output_file}\n')

        if dependency:
            f.write(f'#SBATCH --dependency={dependency}\n')

        if mem > 0:
            f.write(f'#SBATCH --mem={mem}G\n')

        f.write('\n\n')

        f.write("echo `date`\n")
        f.write('echo -e "\\nSLURM_JOBID:\\t\\t" $SLURM_JOBID\n')
        f.write('echo -e "SLURM_JOB_NODELIST:\\t" $SLURM_JOB_NODELIST "\\n"\n')

        if conda_env:
            f.write('module load anaconda\n')
            f.write(f'source activate {conda_env}\n')

        # Handle the commands to run
        if not run_lines:
            raise Exception("No commands were given to be run")

        if isinstance(run_lines, str):
            run_lines = [run_lines]

        if run_lines:
            for line in run_lines:
                f.write(line)
                if not line.endswith('\n'):
                    f.write('\n')

    # Submit the job
    command = ['sbatch', sbatch_file_name, '--parsable']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = process.communicate()

    # stderr contains the message about if GPU is set or not, and stdout contains the ID of the submitted job
    if stderr.startswith('sbatch: GPU Parameter Not Set'):
        if force_gpu:
            raise Exception("GPU was not set when requested")
        else:
            job_id = stdout.split()[-1].strip()
    elif stderr.startswith('sbatch: GPU Parameter Set'):
        job_id = stdout.split()[-1].strip()
    else:
        raise Exception(f"Unknown error from job submission: \n {stderr}")
    return int(job_id)


def sbatch_stp(alg, same_run):
    if same_run:
        jobname = f'stp_{alg}'
        outfile = f'repo/data/stp/{jobname}.out'
        runline = f"./repo/src/bin/release/direction -d stp -h md -a {alg} -i 0-100"
        submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=64, conda_env='HOG2')
    else:
        progress = 0
        total = 100
        for i in range(100):
            jobname = f'stp_{alg}_{i}'
            outfile = f'repo/data/stp/{jobname}.out'
            runline = f"./repo/src/bin/release/direction -d stp -h md -a {alg} -i {i}"
            submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=64, conda_env='HOG2')
            progress += 1
            progress_bar(progress, total)


def sbatch_pancake(gap, alg, same_run):
    if same_run:
        jobname = f'pancake_{gap}_{alg}'
        outfile = f'repo/data/pancake/{jobname}.out'
        runline = f"./repo/src/bin/release/direction -d pancake -h {gap} -a {alg} -i 0-100"
        submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=64, conda_env='HOG2')
    else:
        progress = 0
        total = 100
        for i in range(100):
            jobname = f'pancake_{gap}_{alg}_{i}'
            outfile = f'repo/data/pancake/{jobname}.out'
            runline = f"./repo/src/bin/release/direction -d pancake -h {gap} -a {alg} -i {i}"
            submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=64, conda_env='HOG2')
            progress += 1
            progress_bar(progress, total)


def sbatch_grid(mapname, alg, same_run):
    if mapname is not None:
        map_file = mapname if '.map' in mapname else mapname+'.map'
        jobname = f'grid_{map_file.replace(".map", "")}_{alg}'
        outfile = f'repo/data/grid/{jobname}.out'
        runline = (f"./repo/src/bin/release/direction -d grid -h od -a {alg} -i 0-10000 "
                   f"-m repo/maps/{map_file} -s repo/scenarios/{map_file}.scen")
        submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=16, conda_env='HOG2')
        return
    map_files = [f.name for f in Path('repo/maps').glob('*.map')]
    if same_run:
        jobname = f'grid_{alg}'
        outfile = f'repo/data/grid/{jobname}.out'
        runlines = []
        for map_file in map_files:
            runlines.append(f"./repo/src/bin/release/direction -d grid -h od -a {alg} -i 0-10000 "
                            f"-m repo/maps/{map_file} -s repo/scenarios/{map_file}.scen")
        submit_job(run_lines=runlines, job_name=jobname, output_file=outfile, mem=16, conda_env='HOG2')
    else:
        progress = 0
        total = len(map_files)
        for map_file in map_files:
            jobname = f'grid_{map_file.replace(".map", "")}_{alg}'
            outfile = f'repo/data/grid/{jobname}.out'
            runline = (f"./repo/src/bin/release/direction -d grid -h od -a {alg} -i 0-10000 "
                       f"-m repo/maps/{map_file} -s repo/scenarios/{map_file}.scen")
            submit_job(run_lines=runline, job_name=jobname, output_file=outfile, mem=16, conda_env='HOG2')
            progress += 1
            progress_bar(progress, total)


def parse_args():
    parser = argparse.ArgumentParser(description="Process domain, heuristic, algorithm, and same-file flag.")
    parser.add_argument('--domain', type=str, help="The domain value")
    parser.add_argument('--heuristic', type=str, help="The heuristic value")
    parser.add_argument('--map', type=str, help="The map value")
    parser.add_argument('--alg', type=str, help="The algorithm value")
    parser.add_argument('--same-run', action='store_true', help="Flag for using the same file")

    return parser.parse_args()


def main():
    args = parse_args()
    if args.domain == 'stp':
        sbatch_stp(args.alg, args.same_run)
    elif args.domain == 'grid':
        sbatch_grid(args.map, args.alg, args.same_run)
    elif args.domain == 'pancake':
        sbatch_pancake(args.heuristic, args.alg, args.same_run)
    else:
        print("Unknown Domain")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    sbatch_file_name = 'temp.sh'
    main()
    if Path(sbatch_file_name).exists():
        Path(sbatch_file_name).unlink()
