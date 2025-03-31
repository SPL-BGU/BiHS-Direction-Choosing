<h1 align="center">Direction-selection Policies in BAE*</h1>
<p align="center">
  <a href=""><img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

The code for his repo is based on Prof. Nathan Sturtevant's HOG2 implementation which can be
found [here](https://github.com/nathansttt/hog2). <br/>
The code has been mainly run on the Ubuntu operating system using WSL and natively. <br/>

## Compiling the Code
The code was compiled using GCC and g++ versions 9.4.0. <br/>
You first need to download the dependencies, so run the following command for the headless version of the code.

```sh
# apt install build-essential
```

If you want the non-headless version of HOG2 and the code, or the above does not work, try the following:

```sh
# apt install build-essential libglu1-mesa-dev freeglut3-dev libsfml-dev
```

Then use the compile script in scripts/.

## Running the Experiments
To run the experiments which appeared in the paper, you need to run the following 

```sh
./scripts/toh.sh
./scripts/stp.sh
./scripts/wstp.sh
```
You can also run the main exe with --help flag for more information. 

```sh
./src/bin/release/direction --help
```


## Generating Tables

Once you have all the results, you need to turn them into Excels by running:

```sh
./scripts/analysis.sh
```

Note that analysis.sh will run on all domains, so it might take a minute or two, depending on how many individual log files there are. The more files, the longer it takes (you can simply concat files to speed this up).

The final products are saved into results. The LaTex tables were manually created.

## Known Issues
If you encounter any issue, find a bug, or need help, feel free to open an issue or contact Lior (the maintainer).

## Citation and Code Attribution
The new code for the paper can be found in
[src/paper]().<br/>
As said before, all of these rely on HOG2 which can be found [here](https://github.com/nathansttt/hog2). 

If you find our work interesting or the repo useful, please consider citing this paper:
