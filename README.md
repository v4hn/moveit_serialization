# MoveIt Serialization

Serializer for multiple ros messages including moveit_msgs. Supports yaml serialization through rapidyaml.

## Roadmap
- Add binary serealizer for ros_msgs

## rapidyaml
One of the fastest yaml parser and emiter available. Built internally to avoid unwanted changes to the global error handler.<br>
Current version: https://github.com/captain-yoshi/rapidyaml/commit/2f1f495400043d5f5c9945c815dfdb8668b0f255

## Contribution
The conversion for decoding/encoding moveit_msgs in yaml-cpp was done by Zachary Kingston and taken from the [robowflex](https://github.com/KavrakiLab/robowflex) project.
