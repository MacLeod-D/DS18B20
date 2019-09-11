//#define DEBUG             // Show Debug: Info, Serial Monitor // without: Serial Plotter
#define OneWirePin 13     // Nano,ESP32
#define DEVNUM 5          // max number of devices
double Alpha = 0.95;      // Filter-factor to get smooth curves
double Offset=-2.45;      // compensate Offset for these devices
