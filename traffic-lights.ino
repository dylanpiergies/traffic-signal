#include <Servo.h>

/* Terminology used for variable and function names derived from
   https://www.traffic-signal-design.com/traffic-signal-design-terminology.htm */

constexpr byte ROAD_PHASE_RED_PIN = 11;
constexpr byte ROAD_PHASE_AMBER_PIN = 12;
constexpr byte ROAD_PHASE_GREEN_PIN = 13;

constexpr byte CAR_PARK_PHASE_RED_PIN = 6;
constexpr byte CAR_PARK_PHASE_AMBER_PIN = 7;
constexpr byte CAR_PARK_PHASE_GREEN_PIN = 8;

constexpr byte PEDESTRIAN_PHASE_RED_PIN = 9;
constexpr byte PEDESTRIAN_PHASE_GREEN_PIN = 10;
constexpr byte PEDESTRIAN_WAIT_INDICATOR_PIN = 0;

constexpr byte PEDESTRIAN_REQUEST_SWITCH_PIN = 3;     // Must be a pin that supports interrupts
constexpr byte CAR_PARK_EXIT_REQUEST_SENSOR_PIN = 2;  // Must be a pin that supports interrupts

constexpr byte BARRIER_WARNING_INDICATOR_PIN = 4;
constexpr byte BARRIER_SERVO_CONTROL_PIN = 5;  // Must be a pin that supports PWM
constexpr int BARRIER_SERVO_LOWERED_ANGLE = 0;
constexpr int BARRIER_SERVO_RAISED_ANGLE = 90;
constexpr int BARRIER_SERVO_STEP_DELAY_MS = 15;
constexpr int BARRIER_PREWARN_PERIOD_MS = 1000;

constexpr int ALL_RED_TIME_MS = 2000;
constexpr int MINIMUM_TRAFFIC_GREEN_MS = 7000;
constexpr int PEDESTRIAN_INVITATION_PERIOD_MS = 5000;
constexpr int PEDESTRIAN_POST_INVITATION_PERIOD_MS = 5000;
constexpr int AMBER_TIME_MS = 4000;
constexpr int RED_AMBER_TIME_MS = 3000;

class TrafficPhase {
  byte redPin;
  byte amberPin;
  byte greenPin;

public:
  TrafficPhase(byte redPin, byte amberPin, byte greenPin)
    : redPin{ redPin }, amberPin{ amberPin }, greenPin{ greenPin } {}

  void initialise() {
    pinMode(redPin, OUTPUT);
    pinMode(amberPin, OUTPUT);
    pinMode(greenPin, OUTPUT);

    digitalWrite(redPin, HIGH);
    digitalWrite(amberPin, LOW);
    digitalWrite(greenPin, LOW);
  }

  void changeToGreen() {
    digitalWrite(amberPin, HIGH);
    delay(RED_AMBER_TIME_MS);
    digitalWrite(redPin, LOW);
    digitalWrite(amberPin, LOW);
    digitalWrite(greenPin, HIGH);
  }

  void changeToRed() {
    digitalWrite(greenPin, LOW);
    digitalWrite(amberPin, HIGH);
    delay(AMBER_TIME_MS);
    digitalWrite(amberPin, LOW);
    digitalWrite(redPin, HIGH);
  }
};

class PedestrianPhase {
  byte redPin;
  byte greenPin;

public:
  PedestrianPhase(byte redPin, byte greenPin)
    : redPin{ redPin }, greenPin{ greenPin } {}

  void initialise() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);

    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
  }

  void changeToGreen() {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
  }

  void changeToRed() {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
  }
};

class Barrier {
  byte servoControlPin;
  byte warningLightPin;
  Servo servo;

  void move(int startAngle, int endAngle) {
    digitalWrite(warningLightPin, HIGH);
    delay(BARRIER_PREWARN_PERIOD_MS);
    if (startAngle < endAngle) {
      for (int angle = startAngle; angle <= endAngle; angle++) {
        servo.write(angle);
        delay(BARRIER_SERVO_STEP_DELAY_MS);
      }
    } else {
      for (int angle = startAngle; angle >= endAngle; angle--) {
        servo.write(angle);
        delay(BARRIER_SERVO_STEP_DELAY_MS);
      }
    }
    digitalWrite(warningLightPin, LOW);
  }

public:
  Barrier(byte servoControlPin, byte warningLightPin)
    : servoControlPin{ servoControlPin }, warningLightPin{ warningLightPin } {}

  void initialise() {
    pinMode(warningLightPin, OUTPUT);
    digitalWrite(warningLightPin, LOW);
    servo.attach(servoControlPin);
    lower();
  }

  void raise() {
    move(BARRIER_SERVO_LOWERED_ANGLE, BARRIER_SERVO_RAISED_ANGLE);
  }

  void lower() {
    move(BARRIER_SERVO_RAISED_ANGLE, BARRIER_SERVO_LOWERED_ANGLE);
  }
};

class PedestrianCrossing {
  PedestrianPhase phase;
  byte buttonPin;
  byte waitIndicatorPin;
  volatile bool requested = false;
  volatile bool inviting = false;

public:
  PedestrianCrossing(PedestrianPhase phase, byte buttonPin, byte waitIndicatorPin)
    : phase{ phase }, buttonPin{ buttonPin }, waitIndicatorPin{ waitIndicatorPin } {}

  void initialise() {
    phase.initialise();
    pinMode(buttonPin, INPUT);
    pinMode(waitIndicatorPin, OUTPUT);
  }

  bool isRequested() {
    return requested;
  }

  void runGreenStage() {
    inviting = true;
    requested = false;
    digitalWrite(waitIndicatorPin, LOW);
    phase.changeToGreen();
    delay(PEDESTRIAN_INVITATION_PERIOD_MS);
    inviting = false;
    phase.changeToRed();
    delay(PEDESTRIAN_POST_INVITATION_PERIOD_MS);
  }

  void request() {
    if (!inviting) {
      requested = true;
      digitalWrite(waitIndicatorPin, HIGH);
    }
  }
};

class CarParkExit {
  TrafficPhase phase;
  Barrier barrier;
  byte sensorPin;
  volatile bool requested = false;
  volatile bool inviting = false;

public:
  CarParkExit(TrafficPhase phase, Barrier barrier, byte buttonPin)
    : phase{ phase }, barrier{ barrier }, sensorPin{ buttonPin } {}

  void initialise() {
    phase.initialise();
    barrier.initialise();
    pinMode(sensorPin, INPUT);
  }

  bool isRequested() {
    return requested;
  }

  void runGreenStage() {
    inviting = true;
    requested = false;
    barrier.raise();
    phase.changeToGreen();
    delay(MINIMUM_TRAFFIC_GREEN_MS);
    inviting = false;
    phase.changeToRed();
    barrier.lower();
    delay(ALL_RED_TIME_MS - abs(BARRIER_SERVO_RAISED_ANGLE - BARRIER_SERVO_LOWERED_ANGLE) * BARRIER_SERVO_STEP_DELAY_MS);
  }

  void request() {
    if (!inviting) {
      requested = true;
    }
  }
};

TrafficPhase roadPhase(ROAD_PHASE_RED_PIN, ROAD_PHASE_AMBER_PIN, ROAD_PHASE_GREEN_PIN);
TrafficPhase carParkPhase(CAR_PARK_PHASE_RED_PIN, CAR_PARK_PHASE_AMBER_PIN, CAR_PARK_PHASE_GREEN_PIN);
PedestrianPhase pedestrianPhase(PEDESTRIAN_PHASE_RED_PIN, PEDESTRIAN_PHASE_GREEN_PIN);
Barrier carParkExitBarrier(BARRIER_SERVO_CONTROL_PIN, BARRIER_WARNING_INDICATOR_PIN);
PedestrianCrossing pedestrianCrossing(pedestrianPhase, PEDESTRIAN_REQUEST_SWITCH_PIN, PEDESTRIAN_WAIT_INDICATOR_PIN);
CarParkExit carParkExit(carParkPhase, carParkExitBarrier, CAR_PARK_EXIT_REQUEST_SENSOR_PIN);

void handlePedestrianButtonPressed() {
  pedestrianCrossing.request();
}

void handleCarParkExitRequestSensorActivated() {
  carParkExit.request();
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(PEDESTRIAN_REQUEST_SWITCH_PIN), handlePedestrianButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAR_PARK_EXIT_REQUEST_SENSOR_PIN), handleCarParkExitRequestSensorActivated, FALLING);

  roadPhase.initialise();
  pedestrianCrossing.initialise();
  carParkExit.initialise();

  roadPhase.changeToGreen();
}

void loop() {
  if (pedestrianCrossing.isRequested() || carParkExit.isRequested()) {
    bool pedestrianCrossingStageHasRunThisCycle = false;
    roadPhase.changeToRed();
    delay(ALL_RED_TIME_MS);
    if (pedestrianCrossing.isRequested()) {
      pedestrianCrossing.runGreenStage();
      pedestrianCrossingStageHasRunThisCycle = true;
    }
    if (carParkExit.isRequested()) {
      carParkExit.runGreenStage();
    }
    /* Second opportunity for pedestrian crossing stage to run, i.e. if the pedestrian requested to cross
       while the car park exit phase was running */
    if (pedestrianCrossing.isRequested() && !pedestrianCrossingStageHasRunThisCycle) {
      pedestrianCrossing.runGreenStage();
    }
    roadPhase.changeToGreen();
    delay(MINIMUM_TRAFFIC_GREEN_MS);
  }
}
