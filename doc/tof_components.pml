@startuml
skinparam componentStyle uml2
skinparam monochrome true

package "micro-ROS" {
    [Sensor] as s
    [micro-ROS Client] as urosc
    [micro-ROS Agent] as urosa
    () "SPI" as spi
    () "DDS-XRCE(Serial)" as ddsx
}

[ROS 2] as ros

() "/tof/threshold\n[std_msgs/msg/Int32]" as threshold
() "/tof/trigger\n[std_msgs/msg/Bool]" as trigger
() "/tof/measure\n[std_msgs/msg/Int32]" as meassure
() "/tof/debug\n[std_msgs/msg/Bool]" as debug

ros -down-> debug
ros -down-> threshold
ros <-down- meassure
ros <-down- trigger

urosa <-up- debug
urosa <-up- threshold
urosa -up-> meassure
urosa -up-> trigger

s <-right-> spi
urosc <-left-> spi
urosc <-right-> ddsx
urosa <-left-> ddsx

@enduml