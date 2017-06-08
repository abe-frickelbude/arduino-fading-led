/*
 * A simple blink demo for a bi-color LED that alternates between the
 * colors with a user-definable interval (DURATION below). The signals
 * are output on pins 5 and 6, which will also allow to use PWM for fading instead of
 * simple on/off toggling.
 */
#include <Arduino.h>
#include <HardwareSerial.h>

const uint8_t RED_PIN = 5;
const uint8_t GREEN_PIN = 6;

const uint8_t RED_BRIGHTNESS_DELTA = 1;
const uint8_t GREEN_BRIGHTNESS_DELTA = 1;

const uint8_t MIN_BRIGHTNESS = 0;
const uint8_t MAX_BRIGHTNESS = 200; // with the LED I currently use anything > ~200 produces no further noticeable increase in brightness

const uint16_t BLINK_DURATION = 1000;
const uint16_t FADE_DURATION = 20;

enum LedState {
	INIT,
	RED_RAMP_UP,
	RED_RAMP_DOWN,
	SWITCH_OVER,
	GREEN_RAMP_UP,
	GREEN_RAMP_DOWN
};

// forward function declarations
void setup();
void pingPongBlink();
void pingPongFade();
void printBrightnessValues(const uint8_t& redLedBrightness,
		const uint8_t& greenLedBrightness);

int main(void) {

	init(); // must call this to correctly initialize the core library!
	setup();

	for (;;) {
		//pingPongBlink();
		pingPongFade();
	}
}

void setup() {

	// initialize digital pins for output
	pinMode(RED_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);

	// initialize UART
	Serial.begin(9600);
}

void pingPongBlink() {

	static bool redLedOn = true;

	digitalWrite(RED_PIN, redLedOn);
	digitalWrite(GREEN_PIN, !redLedOn);

	redLedOn = !redLedOn;
	delay(BLINK_DURATION);
}

void printBrightnessValues(const uint8_t& redLedBrightness,
		const uint8_t& greenLedBrightness) {
	Serial.print("[Red:");
	Serial.print(redLedBrightness);
	Serial.print(", Green:");
	Serial.print(greenLedBrightness);
	Serial.println("]");
}

void pingPongFade() {

	// Note: static variables keep their values between method invocations - it's safe to declare
	// them here for better code cohesion!

	// initial state
	static LedState ledState = INIT;

	static uint8_t redLedBrightness = 0;
	static uint8_t greenLedBrightness = 0;

	// ----------------------------  state machine -----------------------------------

	switch (ledState) {

	case INIT:

		// blank both LEDs
		analogWrite(RED_PIN, 0);
		analogWrite(RED_PIN, 0);
		ledState = RED_RAMP_UP;
		break;

	case RED_RAMP_UP:

		analogWrite(RED_PIN, redLedBrightness);

		if (redLedBrightness < MAX_BRIGHTNESS) {
			redLedBrightness += RED_BRIGHTNESS_DELTA;
		} else {
			ledState = RED_RAMP_DOWN;
		}
		break;

	case RED_RAMP_DOWN:

		analogWrite(RED_PIN, redLedBrightness);

		if (redLedBrightness > MIN_BRIGHTNESS) {
			redLedBrightness -= RED_BRIGHTNESS_DELTA;
		} else {
			ledState = SWITCH_OVER;
		}
		break;

	case SWITCH_OVER:

		// turn off PWM to red LED before switching to green
		analogWrite(RED_PIN, 0);
		ledState = GREEN_RAMP_UP;

		break;

	case GREEN_RAMP_UP:

		analogWrite(GREEN_PIN, greenLedBrightness);

		if (greenLedBrightness < MAX_BRIGHTNESS) {
			greenLedBrightness += GREEN_BRIGHTNESS_DELTA;
		} else {
			ledState = GREEN_RAMP_DOWN;
		}
		break;

	case GREEN_RAMP_DOWN:

		analogWrite(GREEN_PIN, greenLedBrightness);

		if (greenLedBrightness > MIN_BRIGHTNESS) {
			greenLedBrightness -= GREEN_BRIGHTNESS_DELTA;
		} else {
			ledState = INIT; // cycle back to initial state
		}
		break;
	}

	//printBrightnessValues(redLedBrightness, greenLedBrightness);
	delay(FADE_DURATION);
}

