#include "FastAccelStepper.h"
#include <cinttypes>
#include "stepper_motors.h"

#define TAG "STEPPER"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

static void setup() {
  engine.init(0);
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    // stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);

#ifdef SUPPORT_ESP32_PULSE_COUNTER
    stepper->attachToPulseCounter(7);
#endif

    printf("Stepper initialized\n");
  } else {
    printf("No stepper\n");
  }
}

void stepper_task() {
  setup();
  int32_t target = 0;
  while (true) {
    while (stepper->isRunning()) {
      //      esp_task_wdt_reset();
      printf("pos=%" PRId32, stepper->getCurrentPosition());
      int16_t pcnt = stepper->readPulseCounter();
      printf("  pcnt=%d", pcnt);
      printf("\n");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("done\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("move\n");
    target = 1000 - target;
    stepper->moveTo(target);
  }
}
