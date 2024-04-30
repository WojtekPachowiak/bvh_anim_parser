# bvh-anim-parser
[![Latest Version]][crates.io] [![Documentation]][docs.rs] ![License]
A Rust library for fast parsing of .bvh files. It not only reads and stores the HIERARCHY and MOTION, but additionaly calculates implicit properties such as global joint positions/rotations, both for the pose (MOTION section) and the rest pose (HIERARCHY section). 


## Usage
See `examples/main.rs` for an exhaustive usecase. 



## Codebase structure
There is clear seperation between types/structs and functions which acts on them.

- `types.rs` contains all the custom structs and types.
- `bvh_parsing.rs` contains all the functions involved in parsing bvh files and getting additional info. from them.
- `skeleton_drawing.rs` is a `bevy` app for visualizing loaded .bvh files. It's purpose was to help me ensure the bvh parser produces sensible results.

There are 2 main structs: `BvhMetadata` and `BvhData`. `BvhData` contains numerical data (in the form of 1D and 2D vectors) of positions and rotations of each joint at each frame and in the rest pose. `BvhMetadata` contains info. such as frame count, fps and joint indices for extracting data out of `BvhData`.

Check out `docs.rs` for details or write an issue asking for more documentation :)


## Convetions
- homogenous 4x4 matrices are represented like this:
```
[ R T ]
[ 0 1 ]
```
where R is 3x3 matrix containg rotation information and NO scaling
where T is 3x1 vector representing translation
- right handed Y-up coordinate system
- joint's local Y-axis points along the bone; local Z-axis is the "forward" direction and local X-axis is the "right" direction.
- when reconstructing rest pose joint orientations:
  - hips' Y points toward 
- I don't differentiate between "orientation" and "rotation" - it's always "rotation".

## What this library is good for:
- getting information out of .bvh files FAST
 
## What this library is NOT for:
- modifying and saving .bvh files

## Assumptions (no warnings/errors will be given if you violate these!):
1. Only one animation (the legend say you can embed multiple animations into a single .bvh) per .bvh file allowed.
2. Hips have translational and rotational components (6 channels), while all the other joints only the rotational one (3 channels). 
3. All rotational components have the same rotation order. For example, if one joint has YZX order while some other has XYZ, then the parser will parse the rotations incorrectly.
4. The non-ENDSITE joint names must not start with the regex 'end*' (case insensitive) - this pattern is used to recognize ENDSITEs.
5. One "\{" or "\}" per line (i.e newlines matter).


## How are rest pose joint rotations calculated?


# Contributing

# License

## Other repos
- https://github.com/Wasserwecken/bvhio (Python) - great .bvh parsing library I've used frequently. But it's not as fast as I would like for large dataset processing. 
- https://github.com/burtonageo/bvh_anim/tree/master (Rust) - highest starred Rust .bvh parser. Seems overengineered and doesn't provide anything that's not already in the HIERARCHY or MOTION section (i.e. doesn't compute rest pose orientations, doesn't do forward kinematics, which allows getting global joint positions/rotations, etc.) 