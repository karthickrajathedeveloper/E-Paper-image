## ESP32 E-Paper Display Project
# Project Overview
This project demonstrates how to display a bitmap image on a Waveshare E-Paper display using an ESP32 microcontroller. The image is fetched over HTTP from a specified URL. The display controller used in this project is the SSD1680.

## Hardware Components
 - ESP32 microcontroller
 - Waveshare E-Paper display (2.66(B), 152x296)

## Software Components
  - Arduino IDE

## Hardware Setup
Connect the ESP32 to the E-Paper display following the pin configuration:
``` CS: Pin 15
DC: Pin 27
RST: Pin 26
BUSY: Pin 25
Power the ESP32 using a suitable power source.
```
## Software Setup
  - Install the Arduino IDE if not already installed.
  - Install the required libraries in the Arduino IDE:
  - WiFi
  - Clone the project repository from GitHub:
 # Copy code
[Code](https://github.com/karthickrajathedeveloper/E-Paper-image/blob/main/E-paper%20display%20image.ino)
## Open the project in the Arduino IDE.
Configuration
# WiFi Configuration
Update the following lines in the code with your WiFi SSID and password:
```
const char* ssid     = "your ssid";
const char* password = "your password";
```
## Display Configuration
The display size and type are configured as follows:

```
 #define GxEPD2_DRIVER_CLASS GxEPD2_266c  // GDEY0266Z90 152x296, SSD1680, (FPC-7510)
```


# helloWorld()
 - Displays the text "HelloWorld" on the E-Paper display.

# showBitmapFrom_HTTP()
 - Fetches and displays a bitmap image from the specified HTTP URL.

# image()
 - Fetches and displays a predefined image from a specified URL.

## Usage
Compile and upload the code to your ESP32 using the Arduino IDE.
Open the Serial Monitor to view debug information and ensure the device is connected to WiFi.
The E-Paper display will show "HelloWorld" followed by the bitmap image fetched from the specified URL.
Troubleshooting
Ensure the WiFi credentials are correct.
Check the connections between the ESP32 and the E-Paper display.
Ensure the server hosting the bitmap image is accessible from the ESP32's network.

## License
This project is licensed under the MIT License.

## Author
[Karthickraja.M](https://github.com/karthickrajathedeveloper).

## GitHub
For more information, visit the [Karthickraja.M](https://github.com/karthickrajathedeveloper).

Feel free to modify the code and the README file to suit your project's needs. Happy coding!
