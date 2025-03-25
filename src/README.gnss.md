# GNSS System Documentation

This document provides comprehensive information about the GNSS (Global Navigation Satellite System) subsystem of the Data Acquisition Digital Twin project. It covers hardware details, RTK functionality, configuration options, and detailed usage instructions.

## Supported GNSS Hardware

The system is designed to work with the following GNSS receivers:

| Receiver Model | Connection Type | Status      | RTK Support | Update Rate |
|----------------|-----------------|-------------|-------------|-------------|
| u-blox F9P     | USB (Serial)    | Supported   | Yes         | Up to 20 Hz |

### Hardware Requirements

- **u-blox F9P**: Requires USB connection or serial adapter
- **GNSS Antenna**: Active antenna recommended for best reception
- **Internet Connection**: Required for NTRIP RTK corrections

## GNSS System Architecture

The GNSS system follows a modular driver-manager-node architecture:

1. **GnssDriver** (`gnss_driver.h/cpp`):
   - Core interface with GNSS hardware
   - Handles serial communication and message parsing
   - Manages hardware configuration and data retrieval

2. **GnssManager** (`gnss_manager.h/cpp`):
   - Manages the GNSS lifecycle
   - Handles ROS2 publishers and subscriptions
   - Processes and filters GNSS data

3. **GnssNode** (`gnss_node.h/cpp`):
   - Implements the ROS2 Node interface
   - Handles parameter management and node configuration
   - Manages component lifecycle

4. **NmeaParser** (`nmea_parser.h/cpp`):
   - Parses NMEA 0183 sentences from the GNSS receiver
   - Extracts position, velocity, and time information
   - Validates checksums and data integrity

5. **RtkProcessor** (`rtk_processor.h/cpp`):
   - Handles RTK correction data
   - Communicates with NTRIP servers for RTCM corrections
   - Feeds correction data to the GNSS receiver

6. **NtripClient** (`ntrip_client.h/cpp`):
   - Implements the NTRIP client protocol
   - Retrieves RTCM correction data from NTRIP casters
   - Handles authentication and connection management

## RTK Functionality Explained

The GNSS system supports RTK (Real-Time Kinematics) for centimeter-level positioning accuracy:

### How RTK Works

1. **Base Station**: A fixed GNSS receiver at a known location
2. **RTCM Corrections**: Base station generates RTCM correction messages
3. **NTRIP Delivery**: Corrections delivered via internet using NTRIP protocol
4. **Rover Application**: Corrections applied to the rover (mobile receiver)
5. **Centimeter Accuracy**: Results in significant accuracy improvement (from meters to centimeters)

### NTRIP Components

- **NTRIP Caster**: Server that distributes RTCM correction data
- **NTRIP Client**: Implemented in the GNSS subsystem to retrieve corrections
- **NTRIP Mountpoint**: Specific correction data stream on the caster
- **RTCM Messages**: Standard format for correction data (typically RTCM 3.x)

## Running the GNSS Subsystem

### Basic GNSS Operation

To run the GNSS node with default settings:

```bash
# Source the ROS2 workspace
source /path/to/data-aquisition-digital-twin/install/setup.bash

# Run GNSS node
ros2 run data_aquisition gnss_node
```

### Running with RTK Enabled

To enable RTK corrections:

```bash
# Run with RTK enabled via command line parameter
ros2 run data_aquisition gnss_node --ros-args -p gnss.use_rtcm_corrections:=true
```

### Using Launch Files

For a more comprehensive setup:

```bash
# Run GNSS with default settings
ros2 launch data_aquisition gnss_launch.py

# Run with RTK enabled
ros2 launch data_aquisition gnss_launch.py use_rtk:=true
```

## GNSS Topics

The GNSS node publishes to the following topics:

| Topic              | Message Type                  | Description                         |
|--------------------|-------------------------------|-------------------------------------|
| `/gnss/fix`        | sensor_msgs/NavSatFix         | Position in latitude/longitude/altitude |
| `/gnss/velocity`   | geometry_msgs/TwistStamped    | Velocity in ENU frame              |
| `/gnss/time`       | sensor_msgs/TimeReference     | GNSS time reference                |
| `/gnss/status`     | diagnostic_msgs/DiagnosticStatus | Receiver status information        |
| `/gnss/rtk_status` | std_msgs/String               | RTK fix status (if enabled)         |

## ROS2 Parameters

The GNSS node accepts the following parameters:

### Basic Parameters

| Parameter                   | Type   | Default              | Description                          |
|-----------------------------|--------|----------------------|--------------------------------------|
| `gnss.model`                | string | "ublox-f9p"          | GNSS receiver model                  |
| `gnss.serial_port`          | string | "/dev/ttyACM0"       | Serial port for GNSS receiver        |
| `gnss.baud_rate`            | int    | 115200               | Serial baud rate                     |
| `gnss.frequency`            | double | 10.0                 | Update frequency in Hz              |
| `gnss.frame_id`             | string | "gnss_frame"         | TF frame ID for GNSS messages        |

### RTK Parameters

| Parameter                       | Type   | Default                  | Description                      |
|---------------------------------|--------|--------------------------|----------------------------------|
| `gnss.use_rtcm_corrections`     | bool   | false                    | Enable RTK corrections           |
| `gnss.rtcm_source`              | string | "NTRIP"                  | Source of RTCM data (NTRIP/Serial) |
| `gnss.ntrip_server`             | string | "caster.example.com:2101/RTCM3" | NTRIP server URL         |
| `gnss.ntrip_username`           | string | ""                       | NTRIP username for authentication |
| `gnss.ntrip_password`           | string | ""                       | NTRIP password for authentication |

### Advanced Parameters

| Parameter                       | Type   | Default                  | Description                      |
|---------------------------------|--------|--------------------------|----------------------------------|
| `gnss.use_dynamic_model`        | bool   | true                     | Enable dynamic model             |
| `gnss.dynamic_model`            | string | "automotive"             | Dynamic model type               |
| `gnss.qos.reliability`          | string | "reliable"               | QoS reliability setting          |
| `gnss.qos.history_depth`        | int    | 10                       | QoS history depth                |

## Configuring RTK

### Step 1: Find a Suitable NTRIP Caster

1. **Public Services**: Many countries offer free NTRIP services through government agencies
2. **Commercial Services**: Subscription-based services with higher reliability
3. **Local Base Station**: Set up your own base station (advanced)

### Step 2: Configure NTRIP Parameters

Edit the `config/gnss/gnss_params.yaml` file:

```yaml
/**:
  ros__parameters:
    gnss:
      use_rtcm_corrections: true
      rtcm_source: "NTRIP"
      ntrip_server: "your-ntrip-server.com:2101/YOUR_MOUNTPOINT"
      ntrip_username: "your_username"  # If required
      ntrip_password: "your_password"  # If required
```

Alternatively, set parameters via command line:

```bash
ros2 run data_aquisition gnss_node --ros-args \
  -p gnss.use_rtcm_corrections:=true \
  -p gnss.ntrip_server:="your-ntrip-server.com:2101/YOUR_MOUNTPOINT" \
  -p gnss.ntrip_username:="your_username" \
  -p gnss.ntrip_password:="your_password"
```

### Step 3: Verify RTK Operation

Check the RTK status topic:

```bash
ros2 topic echo /gnss/rtk_status
```

Look for messages indicating:
- Connected to NTRIP caster
- Receiving RTCM corrections
- RTK fixed or RTK float status

## Modifying GNSS Code

### Changing NMEA Parsing

The NMEA parser can be customized to handle additional sentence types:

```cpp
// In nmea_parser.cpp, add a new sentence handler
bool NmeaParser::parseGxGST(const std::string& sentence) {
    // Parse GST sentence (error statistics)
    // ...
}

// Then register it in the constructor
sentence_parsers_["GST"] = [this](const std::string& s) { return parseGxGST(s); };
```

### Implementing Custom NTRIP Caster Support

For specialized NTRIP services:

```cpp
// In ntrip_client.cpp, modify the request format
std::string NtripClient::createRequest() {
    // Standard request
    std::string req = "GET /" + mountpoint_ + " HTTP/1.0\r\n";
    req += "User-Agent: GNSS_RTK_CLIENT/1.0\r\n";
    
    // Add custom headers for specific services
    req += "X-Custom-Header: Value\r\n";
    
    // Authentication if needed
    if (!username_.empty()) {
        // ... authentication code ...
    }
    
    req += "\r\n";
    return req;
}
```

### Modifying Dynamic Models

To adapt to different vehicle dynamics:

```cpp
// In gnss_driver.cpp
bool GnssDriver::setDynamicModel(const std::string& model) {
    if (model == "automotive") {
        // Configure for automotive use (u-blox specific commands)
        sendUbloxCommand(0x06, 0x24, {0x00, 0x00, 0x00, 0x00, 0x04});
    } else if (model == "airborne") {
        // Configure for airborne use
        sendUbloxCommand(0x06, 0x24, {0x00, 0x00, 0x00, 0x00, 0x06});
    } else if (model == "marine") {
        // Configure for marine use
        sendUbloxCommand(0x06, 0x24, {0x00, 0x00, 0x00, 0x00, 0x05});
    }
    // ... other models ...
}
```

## GNSS Data Visualization and Analysis

### Basic Visualization

```bash
# View raw GNSS fix data
ros2 topic echo /gnss/fix

# View velocity data
ros2 topic echo /gnss/velocity
```

### RViz Integration

For geographic visualization:

```bash
# Make sure the GNSS node is running
ros2 run data_aquisition gnss_node

# Launch RViz2 with GNSS configuration
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix data_aquisition)/share/data_aquisition/config/gnss_view.rviz
```

### Logging and Analysis

To record GNSS data for later analysis:

```bash
# Record all GNSS topics
ros2 bag record /gnss/fix /gnss/velocity /gnss/time /gnss/status /gnss/rtk_status -o gnss_data

# Replay recorded data
ros2 bag play gnss_data
```

## Performance Optimization

For optimal GNSS performance:

1. **Update Rate**: Adjust `gnss.frequency` based on application needs (lower for stability, higher for real-time tracking)
2. **Dynamic Model**: Set appropriate dynamic model for your vehicle type
3. **RTK Settings**: Use the closest available NTRIP mountpoint for best results
4. **Antenna Placement**: Ensure clear sky view and proper antenna mounting

## Common Issues and Solutions

### Communication Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| Cannot connect to serial port | Permission issues, wrong port | Check permissions with `ls -la /dev/ttyACM*`, try `sudo chmod 666 /dev/ttyACM0` |
| No GNSS data | Poor reception, hardware issue | Check antenna connection, ensure outdoor operation or clear sky view |
| Intermittent connection | USB power issues, cable problems | Use powered USB hub, check cable integrity |

### RTK Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| Cannot connect to NTRIP server | Network issue, wrong credentials | Check internet connection, verify server URL and credentials |
| RTK Float but no RTK Fixed | Insufficient correction quality | Ensure base station within range (<50km), check antenna sky view |
| RTCM timeout messages | Network latency, unstable connection | Check internet connection, try different NTRIP caster |

### Accuracy Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| Poor position accuracy | Multipath, poor satellite geometry | Improve antenna placement, check HDOP values |
| Position jumps | Satellite constellation changes | Use smoothing filter, adjust dynamic model |
| Altitude errors | Poor vertical dilution of precision | Focus on 2D position if altitude not critical |

## Real-Time Position Monitoring

Create a simple monitoring script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GnssMonitor(Node):
    def __init__(self):
        super().__init__('gnss_monitor')
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/gnss/fix',
            self.fix_callback,
            10)
        self.rtk_sub = self.create_subscription(
            String,
            '/gnss/rtk_status',
            self.rtk_callback,
            10)
        self.rtk_status = "Unknown"
        
    def fix_callback(self, msg):
        status = "Unknown"
        if msg.status.status == 0:
            status = "No Fix"
        elif msg.status.status == 1:
            status = "Unaugmented Fix" 
        elif msg.status.status == 2:
            status = "RTK Fix"
            
        self.get_logger().info(
            f"Position: {msg.latitude:.7f}, {msg.longitude:.7f}, {msg.altitude:.2f} | "
            f"Status: {status} | RTK: {self.rtk_status} | "
            f"Accuracy: {msg.position_covariance[0]:.4f}m")
    
    def rtk_callback(self, msg):
        self.rtk_status = msg.data

def main():
    rclpy.init()
    monitor = GnssMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save this to a file (e.g., `scripts/gnss/monitor_gnss.py`), make it executable, and run:

```bash
python3 scripts/gnss/monitor_gnss.py
```

## GNSS-Camera Integration

To integrate GNSS with camera data:

1. **Time Synchronization**: Ensure system clock is synchronized with GNSS time
2. **Spatial Registration**: Define transformations between GNSS and camera frames
3. **Fused Applications**: Use position data to georeference camera images or point clouds

Example TF2 static transform:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.5 --yaw 0 --pitch 0 --roll 0 --frame-id gnss_frame --child-frame-id camera_link
```

## Advanced RTK Configuration

### Moving Base RTK

For applications requiring relative positioning between two moving receivers:

```bash
# Run with moving base configuration
ros2 run data_aquisition gnss_node --ros-args -p gnss.operation_mode:=moving_base
```

### Multi-Band Configuration

For u-blox F9P multi-band configuration:

```bash
# Enable GLONASS and Galileo constellations
ros2 run data_aquisition gnss_node --ros-args \
  -p gnss.enable_glonass:=true \
  -p gnss.enable_galileo:=true \
  -p gnss.enable_beidou:=true
```

## Security Considerations

For NTRIP authentication:

1. **Environment Variables**: Store credentials in environment variables rather than in code
2. **Secure Parameters**: Use ROS2 secure parameters when available
3. **Network Security**: Use VPN when connecting to NTRIP servers over public networks

## References

- [u-blox F9P Documentation](https://www.u-blox.com/en/product/zed-f9p-module)
- [NTRIP Protocol Documentation](https://gssc.esa.int/wp-content/uploads/2018/07/NtripDocumentation.pdf)
- [RTK Principles](https://www.trimble.com/positioning-services/pdf/WhitePaper_RTK_GNSS_Principles.pdf)
- [ROS2 NavSatFix Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)