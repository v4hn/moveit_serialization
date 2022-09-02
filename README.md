# MoveIt Serialization

Serializer for multiple ros messages including moveit_msgs. Supports yaml serialization through rapidyaml.

## Roadmap
- Add binary serealizer for ros_msgs

## rapidyaml
One of the fastest yaml parser and emiter available. Built internally to avoid unwanted changes to the global error handler.<br>
Current version: https://github.com/captain-yoshi/rapidyaml/commit/2ee74c1d940ff61bfc1afc357ec5f28113e6a325

## Contribution
The conversion for decoding/encoding moveit_msgs in yaml-cpp was done by Zachary Kingston and taken from the [robowflex](https://github.com/KavrakiLab/robowflex) project.
