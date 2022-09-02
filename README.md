# MoveIt Serialization

Serializer for moveit_msgs. Currently supports yaml serialization through yaml-cpp and rapidyaml.


## Roadmap
- Add binary serealizer for ros_msgs
- Deprecate yaml-cpp in favor of rapidyaml

## rapidyaml
**[BETA]** One of the fastest yaml parser and emiter available. Built internally to avoid unwanted changes to the global error handler.<br>
Current version: https://github.com/captain-yoshi/rapidyaml/commit/2ee74c1d940ff61bfc1afc357ec5f28113e6a325


## yaml-cpp

The package [moveit\_serialisation\_yamlcpp](./yaml-cpp) integrates the [yaml-cpp](https://github.com/jbeder/yaml-cpp) library which is a [YAML](http://www.yaml.org/) parser and emitter in C++ matching the [YAML 1.2 spec](http://www.yaml.org/spec/1.2/spec.html).


RViz depends on the `yaml-cpp` library. The [MoveIt Benchmark Suite](https://github.com/captain-yoshi/moveit_benchmark_suite) (MBS) also uses heavily this library and also depends on RVIz. MBS needs a minimal version of `0.6.3`. When installing RViz on Ubuntu 20.04, via the package manager, the *yaml-cpp* version defaults to `0.6.2`. Runtime errors were trigerred because the MBS executable was linking to multiple different versions of *yaml-cpp* library. This is why the library was integrated as a ROS package.


## Contribution

The yaml-cpp conversion for decoding/encoding moveit_msgs was done by Zachary Kingston and taken from the [robowflex](https://github.com/KavrakiLab/robowflex) project.
