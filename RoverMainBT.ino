#include <Bluepad32.h>
#include <CtrlUtil.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Define the BTS7960 control pins
const int RPWM1 = 26; // Right PWM pin
const int LPWM1 = 27; // Left PWM pin
const int RPWM2 = 12; // Right PWM pin
const int LPWM2 = 14; // Left PWM pin
// const int R_EN = 2; // Right Enable pin
// const int L_EN = 4; // Left Enable pin
int joy[4];
float pLeft, pRight, pwr, pwr1, pwr2 = 0;

void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Set the control pins as outputs
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);

    stopMotors();
 }

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    stopMotors();
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}


void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}
void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // % power to each track
    pLeft = remap(ctl->throttle(), 0, 1023, 0, 100);
    pRight = remap(ctl->brake(), 0, 1023, 0, 100);
    pLeft = 100 - pLeft;
    pRight = 100 - pRight;

    // total power
    pwr1 = remap(ctl->axisRY(), -511, 512, -127, 128);
    pwr2 = remap(ctl->axisY(), -511, 512, -127, 128);
    // deadzone
    if(pwr1 < 10 and pwr1>-10) {
        pwr1 = 0;
    }
    if(pwr2 < 10 and pwr2>-10) {
        pwr2 = 0;
    }

    if(pwr1 > 0 and pwr2 >= 0){
        pwr = pwr1 + pwr2;
        joy[0] = pwr*(pLeft/100);
        joy[1] = pwr*(pRight/100);
        joy[2] = 0;
        joy[3] = 0;
    } else if(pwr1 < 0 and pwr2 <= 0){
        pwr = (-pwr1) + (-pwr2);
        pwr = pwr;
        joy[0] = 0;
        joy[1] = 0;
        joy[2] = pwr*(pLeft/100);
        joy[3] = pwr*(pRight/100);
    } else if(pwr1 > 0 and pwr2 < 0){
        joy[0] = pwr1*(pLeft/200);
        joy[1] = 0;
        joy[2] = 0;
        joy[3] = -pwr2*(pRight/200);
    } else if(pwr1 < 0 and pwr2 > 0){
        joy[0] = 0;
        joy[1] = -pwr1*(pRight/200);
        joy[2] = pwr2*(pLeft/200);
        joy[3] = 0;
    } else {
        // should be dead condition now after all cases are implemented
        stopMotors();
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}
void stopMotors(){
    Serial.println("Stop");
    joy[0] = 0;
    joy[1] = 0;
    joy[2] = 0;
    joy[3] = 0;
}
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1);

    if(dataUpdated){
      processControllers();
      Serial.print("Received: ");
      Serial.print(joy[0]);
      Serial.print(", ");
      Serial.print(joy[1]);
      Serial.print(", ");
      Serial.print(joy[2]);
      Serial.print(", ");
      Serial.println(joy[3]);
    }

    analogWrite(RPWM1, joy[0]); // speed forward
    analogWrite(RPWM2, joy[1]); // speed forward
    
    analogWrite(LPWM1, joy[2]); // speed reverse
    analogWrite(LPWM2, joy[3]); // speed reverse

}