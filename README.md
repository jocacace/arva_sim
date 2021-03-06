# arva_sim
This ROS package provides the implementation of a Gazebo plugins to simulate the behavior of an [ARVA](https://en.wikipedia.org/wiki/Avalanche_transceiver) transceiver systems. ARVA means _Appareil de Recherche de  Victims en  Avalanche_ and represents the forefront technology in rescuing avalanche victims. In particular, the proposed package contains two different plugins: the _receiver_ and _transmitter_. 

### Background
ARVA system is one of the forefront technologies nowadays used in _Search & Rescue_ operations in case of avalanches. The ARVA devices consist of two main elements, i.e. a transmitter and a receiver, which are operated alternatively. Excursionists and skiers who use the ARVA normally set in the transmitting mode so that, in the accidental case of avalanche, the system is already set in the right operative mode. In the receiver mode, the ARVA devices provide information about the electromagnetic field, emitted by the transmitter, which is exploited to guide the rescuer toward the victim. 
The working principle of this sensor is based on the detection of the magnetic low-power pulses, emitted by the transmitter. This information is usually made available to the rescuers in terms of magnetic distance and magnetic direction to the victim. 

This package contains two Gazebo ROS plugins. One implementing the ARVA transmitter and another one the receiver. Using this plugins, researcher and developer can investigate new _Search & Rescue_ strategies using ARVA data, endowing simulated robots with the ARVA receiver.

### Documentation
To install and test the _arva\_sim_ package you can refer to the [wiki](https://github.com/jocacace/arva_sim/wiki/ARVA-Gazebo-ROS-plugins) page of the project


