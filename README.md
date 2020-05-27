# Rodent

Rodent is a BVH traversal library and renderer implemented using the AnyDSL compiler framework (https://anydsl.github.io/).

# Spectral fork

This fork is an experimental extension of the original Rodent with spectral raytracing.


The spectral raytracing is achieved with Hero Wavelength spectral sampling:

A. Wilkie, S. Nawaz, M. Droske, A. Weidlich, and J. Hanika. 2014. Hero wavelength spectral sampling. In Proceedings of the 25th Eurographics Symposium on Rendering (EGSR ’14). Eurographics Association, Goslar, DEU, 123–131. DOI:https://doi.org/10.1111/cgf.12419


Upsampling of existent sRGB data is done with A Low-Dimensional Function Space for Efficient Spectral Upsampling:

Wenzel Jakob and Johannes Hanika. 2019. A Low-Dimensional Function Space for Efficient Spectral Upsampling. In Computer Graphics Forum (Proceedings of Eurographics) 38(2).

# Building

The dependencies are: CMake, AnyDSL, libpng, SDL2, and optionally the Embree sources for the benchmarking tools.
Once the dependencies are installed, use the following commands to build the project:

    mkdir build
    cd build
    # Set the OBJ file to use with the SCENE_FILE variable
    # By default, SCENE_FILE=../testing/cornell_box.obj
    cmake .. -DSCENE_FILE=myfile.obj
    # Optional: Create benchmarking tools for Embree and BVH extractor tools
    # cmake .. -DEMBREE_ROOT_DIR=<path to Embree sources>
    make

# Testing

This section assumes that the current directory is the build directory. To run rodent, just type:

    bin/rodent

You may want to change the initial camera parameters using the command line options `--eye`, `--dir` and `--up`. Run `bin/rodent --help` to get a full list of options.

When ImageMagick is found by Cmake, use the following commands to test the traversal code with the provided test scene:

    make test

This will only test the primary ray distribution with the packet, single, and hybrid variants.
To test all possible combinations, or if you do not have ImageMagick installed, use the benchmarking tool directly:

    bin/bench_traversal -bvh ../testing/sponza.bvh -ray ../testing/sponza-primary.rays --bench 50 --warmup 10 --tmax 5000 -o output-hybrid-primary.fbuf
    bin/bench_traversal -bvh ../testing/sponza.bvh -ray ../testing/sponza-primary.rays --bench 50 --warmup 10 --tmax 5000 -s -o output-single-primary.fbuf
    bin/bench_traversal -bvh ../testing/sponza.bvh -ray ../testing/sponza-random.rays --bench 50 --warmup 10 --tmax 1 -o output-hybrid-random.fbuf
    bin/bench_traversal -bvh ../testing/sponza.bvh -ray ../testing/sponza-random.rays --bench 50 --warmup 10 --tmax 1 -s -o output-single-random.fbuf
    bin/fbuf2png -n output-hybrid-primary.fbuf output-hybrid-primary.png
    bin/fbuf2png -n output-single-primary.fbuf output-single-primary.png
    bin/fbuf2png -n output-hybrid-random.fbuf output-hybrid-random.png
    bin/fbuf2png -n output-single-random.fbuf output-single-random.png

This will run the traversal on the test set, and generate images as a result. For the primary ray distribution, the _hybrid_ and _single_ variants should generate the same images. The reference images for primary and random rays are in the `testing` directory.

Running `bin/bench_traversal --help` will provide a list of additional options.
