# v2i_command_muxer

## Orverview
This is a node that muxer the ros2 topic by Autoware and the ros2 topic of the cargo loading service.

## Input and Outoput
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/awapi/tmp/infrastructure_commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
  - from [cargo_loading_service](https://github.com/eve-autonomy/cargo_loading_service)
    - `/cargo_loding/infurastructre_commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
- output
  - to [v2i_interface](https://github.com/eve-autonomy/v2i_interface)
    - `/v2i/infrastructure_commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
