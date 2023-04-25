# USB Power Delivery for Arduino

Implement a USB PD protocol analyzer, a USB PD trigger board or a more
sophisticated power sink using with a few additional components and simple
Arduino code. Supports several STM32 microcontrollers.

Dependning on the microcontroller, a comparator and a few resistors are needed, or only resistors or no additional component at all. See below for more details. For 5 USD in parts, you can build a USB PD protocol analyzer.



## Supported Boards

| Board | Required additional components |
| - | - |
| Blue Pill (STM32F103C8) | Dual comparator, several resistors |
| Black Pill (STM32F401CC) | Dual comparator, several resistors |
| Nucleo-L432KC | Several resistors (for power sink) or none (for protocol analyzer) |
| Nucleo-G431KB | None |

All boards require an additional USB C connector as the standard connector is not ready for USB Power Delivery (no USB C connector, CC1/CC2 signals not available, voltage regular cannot handle more than 5V).

See the Wiki for how to wire the board and the additional components.


## Library Installation (Arduino IDE)

1. In the Arduino IDE, navigate to *Sketch > Include Library > Manage Libraries...*

2. The Library Manager will open and you will find a list of libraries that are already installed or ready for installation.

3. Search for *Power Delivery* using the search bar.

4. Click on the *INSTALL* button to install it.



## Examples

### Protocol Analyzer

The protocol analyzer can be connected between two USB PD devices to monitor the USB PD communication.

```c++
#include "PowerDelivery.h"

void setup() {
  Serial.begin(115200);
  PowerController.startMonitor();
}

void loop() {
  ProtocolAnalyzer::poll();
}
```

See the Wiki for details regarding the required components and wiring.



### Trigger Board

The trigger boards communicates with a USB power supply and requests a different voltage than
the initial 5V.

```c++
#include "PowerDelivery.h"

void setup() {
  PowerSink.start();
  // request 12V @ 1A once power supply is connected
  PowerSink.requestPower(12000, 1000);
}

void loop() {
  // nothing to do
}
```

See the Wiki for details regarding the required components and wiring.
