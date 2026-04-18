
This project focuses on designing and implementing a wall-following autonomous mobile robot capable of navigating a predefined track without human intervention. The robot follows walls using distance sensors and accurately handles sharp 90° turns while maintaining stability and alignment within the track.

A key feature of the system is its ability to detect each turn, classify it as left or right, and maintain a count of all turns encountered during execution. This data is then transmitted to a PC using WiFi or Bluetooth in a structured format for verification and analysis.

The project emphasizes low-level embedded C programming without relying on high-level Arduino libraries. It incorporates real-time control techniques, sensor interfacing, and a Finite State Machine (FSM) to manage robot behavior, including wall tracking, turn detection, and recovery from edge cases such as losing the wall.

The system integrates mechanical design, electrical components, and software logic into a complete embedded solution. The overall objective is to complete the track autonomously in the shortest time while ensuring accurate turn detection and reliable communication.
