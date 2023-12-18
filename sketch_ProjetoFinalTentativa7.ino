#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"
#include <Wire.h>

//-------Definição de pinos e constantes-----//
#define mainDELAY_LOOP_COUNT 400000  // ( 0xffffff )
#define LM35_PIN 35
#define pinMotorControl 4
#define motorPin 32
#define mainSENDER_1 1
#define mainSENDER_2 2
#define LDR_PIN 34
#define servoPin 15
#define pinServoControl 4
#define ADC_RESOLUTION 12
#define VREF_PLUS 3.3
#define VREF_MINUS 0.0


/* Definição de funções de tasks*/
static void vSenderTask(void *pvParameters);
static void vReceiverTask(void *pvParameters);
void vTaskTemp(void *pvParameters);
void vTaskLuminusidade(void *pvParameters);
void vTaskServoON(void *pvParameters);
void vTaskServoOFF(void *pvParameters);
void vTaskHumidity(void *pvParameters);
void vTaskServoAuxiliar(void *pvParameters);
void servopulse(int myangle);
bool my_vApplicationIdleHook(void);

/* Definicao das funcoes para os semaforos */
static void IRAM_ATTR vInterruptMotorON(void);
static void IRAM_ATTR vInterruptMotorOFF(void);
static void vHandlerTaskMotorON(void *pvParameters);
static void vHandlerTaskMotorOFF(void *pvParameters);

/* Definição das tasks Handels */
TaskHandle_t xTaskLM35Handle;
TaskHandle_t xTaskLuminusidadeHandle;
TaskHandle_t xTaskMotorHandle;
TaskHandle_t xTaskHumHandle;
TaskHandle_t xTaskServoONHandle;
TaskHandle_t xTaskServoOFFHandle;

/* Definição das queues */
QueueHandle_t xQueue;
QueueHandle_t xServoQueue;

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xSemaforoMotorON;
SemaphoreHandle_t xSemaforoMotorOFF;

// pin to generate interrupts
const uint8_t interruptPin = 14;
const uint8_t interruptPin2 = 12;
unsigned int op = 0;

/* Define the structure type that will be passed on the queue. */
typedef struct
{
  unsigned int ucOp;
  unsigned char ucSource;
} xData;

/* Declare two variables of type xData that will be passed on the queue. */

static const xData xStructsToSend[2] = {
  { op, mainSENDER_1 }, /* Used by Sender1. */
  { op, mainSENDER_2 }  /* Used by Sender2. */
};

void setup(void) {
  // Set loopTask max priority before deletion
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
  esp_register_freertos_idle_hook(my_vApplicationIdleHook);  // Register idle hook callback
  // Init USART and set Baud-rate to 115200
  Serial.begin(115200);
  Wire.begin(); //Inicializaçao I2C
  analogReadResolution(ADC_RESOLUTION);
  xSemaforoMotorON = xSemaphoreCreateCounting(10, 0);
  xSemaforoMotorOFF = xSemaphoreCreateCounting(10, 0);
  while (!Serial) {
    continue; 
  }
  /* The queue is created to hold a maximum of 3 structures of type xData. */
  xQueue = xQueueCreate(3, sizeof(xData));
  xServoQueue = xQueueCreate(5, sizeof(int));

  /*Tasks de sensores podem ser definidas fora do if da xQueue*/
  xTaskCreatePinnedToCore(vTaskServoAuxiliar, "Servo motor", 1024, NULL, 2, NULL, 1);

  pinMode(pinMotorControl, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);
  pinMode(interruptPin, OUTPUT);
  pinMode(interruptPin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), &vInterruptMotorON, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), &vInterruptMotorOFF, RISING);

  if (xSemaforoMotorON != NULL) {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreatePinnedToCore(vHandlerTaskMotorON, "Handler", 1024, NULL, 3, NULL, 1);

    /* Create the task that will periodically generate a software interrupt.
    This is created with a priority below the handler task to ensure it will
    get preempted each time the handler task exist the Blocked state. */

    /* Start the scheduler so the created tasks start executing. */
    //vTaskStartScheduler();
  }

  if (xSemaforoMotorOFF != NULL) {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreatePinnedToCore(vHandlerTaskMotorOFF, "Handler", 1024, NULL, 3, NULL, 1);

    /* Create the task that will periodically generate a software interrupt.
    This is created with a priority below the handler task to ensure it will
    get preempted each time the handler task exist the Blocked state. */

    /* Start the scheduler so the created tasks start executing. */
    //vTaskStartScheduler();
  }

  if (xQueue != NULL) {
    /* Create two instances of the task that will write to the queue.  The
    parameter is used to pass the structure that the task should write to the
    queue, so one task will continuously send xStructsToSend[ 0 ] to the queue
    while the other task will continuously send xStructsToSend[ 1 ].  Both
    tasks are created at priority 2 which is above the priority of the receiver. */
    xTaskCreatePinnedToCore(vSenderTask, "Sender1", 1024, (void *)&(xStructsToSend[0]), 2, NULL, 1);
    xTaskCreatePinnedToCore(vSenderTask, "Sender2", 1024, (void *)&(xStructsToSend[1]), 2, NULL, 1);

    /* Create the task that will read from the queue.  The task is created with
    priority 1, so below the priority of the sender tasks. */
    xTaskCreatePinnedToCore(vReceiverTask, "Receiver", 1024, NULL, 1, NULL, 1);

    /* Start the scheduler so the created tasks start executing. */
    //vTaskStartScheduler();
  } else {
    /* The queue could not be created. */
  }

  /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
  //  for( ;; );
  //  return 0;
}
/*-----------------------------------------------------------*/

static void vSenderTask(void *pvParameters) {
  portBASE_TYPE xStatus;
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  const TickType_t xDelay100ms = 100 / portTICK_PERIOD_MS;
  String stringRecebida3;  // variavel com os dados de entrada
  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {

    Serial.println(" \r\n Escolha uma operação a realizar:  ");
    Serial.println("1 -> Mostrar a temperatura");
    Serial.println("2 -> Mostrar a luminusidade");
    Serial.println("3 -> Acionar o motor");
    Serial.println("4 -> Acionar a conduta");
    Serial.println("5 -> Desacionar o motor");
    Serial.println("6 -> Desacionar a conduta");
    Serial.println("7 -> Mostrar a humidade");

    while (!Serial.available()) {
      continue;                       //Espera que o user aperte o enter com a opcao escolhida
    }
    stringRecebida3 = Serial.readStringUntil('\n'); 
    op = stringRecebida3.toInt(); 

    if (op == 1) {

      Serial.println("\r\nTemperatura: ");  //imprime no ecrã serie a mensagem que está entre aspas
      xTaskCreatePinnedToCore(vTaskTemp, "TaskTemp", 1024, NULL, 2, &xTaskLM35Handle, 1);
    }

    else if (op == 2) {

      Serial.println("\r\nLuminusidade:");  //imprime no ecrã serie a mensagem que está entre aspas
      xTaskCreatePinnedToCore(vTaskLuminusidade, "TaskLuminusidade", 1024, NULL, 2, &xTaskLuminusidadeHandle, 1);
    }

    else if (op == 3) {
      Serial.println("\r\nMotor ON:");  //imprime no ecrã serie a mensagem que está entre aspas
      Serial.print("EX13: Periodic task - About to generate an interrupt.\r\n");
      digitalWrite(interruptPin, LOW);
      digitalWrite(interruptPin, HIGH);
      Serial.print("EX13: Periodic task - Interrupt generated.\r\n\r\n\r\n");
    }

    else if (op == 4) {
      Serial.println("\r\nConduta ativa :");  //imprime no ecrã serie a mensagem que está entre aspas
      xTaskCreatePinnedToCore(vTaskServoON, "TaskServo", 1024, NULL, 3, &xTaskServoONHandle, 1);
    }

    else if (op == 5) {
      Serial.println("\r\nMotor OFF :");  //imprime no ecrã serie a mensagem que está entre aspas
      Serial.print("EX13: Periodic task - About to generate an interrupt.\r\n");
      digitalWrite(interruptPin2, LOW);
      digitalWrite(interruptPin2, HIGH);
      Serial.print("EX13: Periodic task - Interrupt generated.\r\n\r\n\r\n");
    }

    else if (op == 6) {
      Serial.println("\r\nConduta desativada :");  //imprime no ecrã serie a mensagem que está entre aspas
      xTaskCreatePinnedToCore(vTaskServoOFF, "TaskServo2", 1024, NULL, 3, &xTaskServoOFFHandle, 1);
    }

    else if (op == 7) {

      Serial.println("\r\nHumidade :");  //imprime no ecrã serie a mensagem que está entre aspas
      xTaskCreatePinnedToCore(vTaskHumidity, "TaskHumidity", 2048, NULL, 2, &xTaskHumHandle, 1);
    }

    else if (op != 1 || op != 2 || op != 3 || op != 4 || op != 5 || op != 6 || op != 7) {

      Serial.println("\r\nTêm de inserir uma opção correta");  //imprime no ecrã serie a mensagem que está entre aspas
    }
    /* The first parameter is the queue to which data is being sent.  The
    queue was created before the scheduler was started, so before this task
    started to execute.

    The second parameter is the address of the structure being sent.  The
    address is passed in as the task parameter.

    The third parameter is the Block time - the time the task should be kept
    in the Blocked state to wait for space to become available on the queue
    should the queue already be full.  A block time is specified as the queue
    will become full.  Items will only be removed from the queue when both
    sending tasks are in the Blocked state.. */
    xStatus = xQueueSendToBack(xQueue, pvParameters, xTicksToWait);

    if (xStatus != pdPASS) {
      /* We could not write to the queue because it was full - this must
      be an error as the receiving task should make space in the queue
      as soon as both sending tasks are in the Blocked state. */
      Serial.print("EX11: Could not send to the queue.\r\n");
      vTaskDelay(xDelay100ms);
    }

    else if (xStatus == pdPASS) {
      /* We could not write to the queue because it was full - this must
      be an error as the receiving task should make space in the queue
      as soon as both sending tasks are in the Blocked state. */
      Serial.print("EX11: Sended to the queue.\r\n");
      vTaskDelay(xDelay100ms);
    }

    /* Allow the other sender task to execute. */
    taskYIELD();
  }
}
/*-----------------------------------------------------------*/

static void vReceiverTask(void *pvParameters) {
  /* Declare the structure that will hold the values received from the queue. */
  xData xReceivedStructure;
  portBASE_TYPE xStatus;

  /* This task is also defined within an infinite loop. */
  for (;;) {
    /* As this task only runs when the sending tasks are in the Blocked state,
    and the sending tasks only block when the queue is full, this task should
    always find the queue to be full.  3 is the queue length. */
    if (uxQueueMessagesWaiting(xQueue) != 3) {
      Serial.print("EX11: Queue should have been full!\r\n");
    }

    /* The first parameter is the queue from which data is to be received.  The
    queue is created before the scheduler is started, and therefore before this
    task runs for the first time.

    The second parameter is the buffer into which the received data will be
    placed.  In this case the buffer is simply the address of a variable that
    has the required size to hold the received structure.

    The last parameter is the block time - the maximum amount of time that the
    task should remain in the Blocked state to wait for data to be available
    should the queue already be empty.  A block time is not necessary as this
    task will only run when the queue is full so data will always be available. */
    xStatus = xQueueReceive(xQueue, &xReceivedStructure, 0);

    if (xStatus == pdPASS) {
      /* Data was successfully received from the queue, print out the received
      value and the source of the value. */
      if (xReceivedStructure.ucSource == mainSENDER_1) {
        Serial.print("EX11: From Sender 1 = ");
        Serial.println(xReceivedStructure.ucOp);
      } else {
        Serial.print("EX11: From Sender 2 = ");
        Serial.println(xReceivedStructure.ucOp);
      }
    } else {
      /* We did not receive anything from the queue.  This must be an error
      as this task should only run when the queue is full. */
      Serial.print("EX11: Could not receive from the queue.\r\n");
    }
  }
}

static void vHandlerTaskMotorON(void *pvParameters) {
  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake(xSemaforoMotorON, portMAX_DELAY);

    /* To get here the event must have occurred.  Process the event (in this
    case we just print out a message). */

    for (int x = 0; x <= 255; x++) {
      digitalWrite(motorPin, 255);
      Serial.print("Motor ON \r\n");
    }
  }
}

static void vHandlerTaskMotorOFF(void *pvParameters) {
  /* As per most tasks, this task is implemented within an infinite loop. */
  for (;;) {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake(xSemaforoMotorOFF, portMAX_DELAY);

    /* To get here the event must have occurred.  Process the event (in this
    case we just print out a message). */


    for (int x = 255; x >= 0; x--) {
      digitalWrite(motorPin, 0);
      Serial.print("Motor OFF \r\n");
    }
  }
}

static void IRAM_ATTR vInterruptMotorON(void) {
  static portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore multiple times.  The first will unblock the handler
  task, the following 'gives' are to demonstrate that the semaphore latches
  the events to allow the handler task to process them in turn without any
  events getting lost.  This simulates multiple interrupts being taken by the
  processor, even though in this case the events are simulated within a single
  interrupt occurrence.*/
  xSemaphoreGiveFromISR(xSemaforoMotorON, (BaseType_t *)&xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemaforoMotorON, (BaseType_t *)&xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemaforoMotorON, (BaseType_t *)&xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    /* Giving the semaphore unblocked a task, and the priority of the
    unblocked task is higher than the currently running task - force
    a context switch to ensure that the interrupt returns directly to
    the unblocked (higher priority) task.

    NOTE: The syntax for forcing a context switch is different depending
    on the port being used.  Refer to the examples for the port you are
    using for the correct method to use! */
    portYIELD_FROM_ISR();
    //vPortYield();
  }
}

static void IRAM_ATTR vInterruptMotorOFF(void) {
  static portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore multiple times.  The first will unblock the handler
  task, the following 'gives' are to demonstrate that the semaphore latches
  the events to allow the handler task to process them in turn without any
  events getting lost.  This simulates multiple interrupts being taken by the
  processor, even though in this case the events are simulated within a single
  interrupt occurrence.*/
  xSemaphoreGiveFromISR(xSemaforoMotorOFF, (BaseType_t *)&xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemaforoMotorOFF, (BaseType_t *)&xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemaforoMotorOFF, (BaseType_t *)&xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    /* Giving the semaphore unblocked a task, and the priority of the
    unblocked task is higher than the currently running task - force
    a context switch to ensure that the interrupt returns directly to
    the unblocked (higher priority) task.

    NOTE: The syntax for forcing a context switch is different depending
    on the port being used.  Refer to the examples for the port you are
    using for the correct method to use! */
    portYIELD_FROM_ISR();
    //vPortYield();
  }
}

/* ----- Idle Hook Callback ----- */
bool my_vApplicationIdleHook(void) {
  return true;
}

//------------------------------------------------------------------------------
void loop() {
  vTaskDelete(NULL);
}

void vTaskTemp(void *pvParameters) {
  xData xReceivedStructure;
  const TickType_t xDelay200ms = 200 / portTICK_PERIOD_MS;
  volatile unsigned long ul;
  float temp_adc_val = 0;
  float temp_val = 0;
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;) {
    while (xQueueReceive(xQueue, &xReceivedStructure, 0) != errQUEUE_EMPTY) {

      temp_adc_val = analogRead(LM35_PIN) * (2.5 / 1024.0);
      temp_val = (temp_adc_val / 0.01);
      Serial.print("\r\n Temperatura LM35 :");
      Serial.print(temp_val);
      vTaskDelay(xDelay200ms);
      vTaskDelete(xTaskLM35Handle);

      /* Delay for a period. */
      for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
        /* This loop is just a very crude delay implementation.  There is
      nothing to do in here.  Later exercises will replace this crude
      loop with a proper delay/sleep function. */
      }
    }
  }
}

void vTaskLuminusidade(void *pvParameters) {
  xData xReceivedStructure;
  const TickType_t xDelay200ms = 200 / portTICK_PERIOD_MS;
  volatile unsigned long ul;
  int analog_value = 0;
  float analog_voltage = 0;
  float valor_lux = 0;
  float percentagem = 0;
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;) {

    while (xQueueReceive(xQueue, &xReceivedStructure, 0) != errQUEUE_EMPTY) {

      analog_value = analogRead(LDR_PIN);
      analog_voltage = analog_value * (VREF_PLUS - VREF_MINUS) / (pow(2.0, (float)ADC_RESOLUTION)) + VREF_MINUS;
      percentagem = (analog_voltage * 100) / 3.3;
      Serial.print("\n Percentagem de sombra: ");
      Serial.println(percentagem);
      Serial.print("ADC_1_0 VOLT: ");
      Serial.println(analog_voltage);
      vTaskDelay(xDelay200ms);
      vTaskDelete(xTaskLuminusidadeHandle);

      for (ul = 0; ul < mainDELAY_LOOP_COUNT; ul++) {
        /* This loop is just a very crude delay implementation.  There is
      nothing to do in here.  Later exercises will replace this crude
      loop with a proper delay/sleep function. */
      }
    }
  }
}


void vTaskServoON(void *pvParameters) {
  xData xReceivedStructure;
  int pos_servo;
  const TickType_t xDelay200ms = 200 / portTICK_PERIOD_MS;
  servopulse(0);

  for (int i = 0; i <= 15; i++) {
    servopulse(90);  // Move servo to position in variable 'i'
  }
  for (;;) {

    while (xQueueReceive(xQueue, &xReceivedStructure, 0) != errQUEUE_EMPTY) {
      if (1) {
        servopulse(90);
        Serial.println("Conduta aberta");
        vTaskDelay(xDelay200ms);
        vTaskDelete(xTaskServoONHandle);
      }
    }
  }
}

void vTaskServoOFF(void *pvParameters) {
  xData xReceivedStructure;
  int pos_servo;
  servopulse(0);
  const TickType_t xDelay200ms = 200 / portTICK_PERIOD_MS;
  for (int i = 0; i <= 15; i++) {
    servopulse(0);  // Move servo to position in variable 'i'
  }
  for (;;) {
    while (xQueueReceive(xQueue, &xReceivedStructure, 0) != errQUEUE_EMPTY) {
      if (1) {
        servopulse(0);
        Serial.println("Conduta fechada");
        vTaskDelay(xDelay200ms);
        vTaskDelete(xTaskServoOFFHandle);
      }
    }
  }
}

void vTaskHumidity(void *pvParameters) {
  xData xReceivedStructure;
  const TickType_t xDelay200ms = 200 / portTICK_PERIOD_MS;
  for (;;) {

    while (xQueueReceive(xQueue, &xReceivedStructure, 0) != errQUEUE_EMPTY) {

      int firstHumiByte = 0, secondHumiByte = 0, thirdHumiByte = 0;
      Wire.beginTransmission(0x40);
      Wire.write(0xF5);
      Wire.endTransmission();
      vTaskDelay(xDelay200ms);
      Wire.requestFrom(0x40, 3);
      while (Wire.available() > 2) {
        firstHumiByte = (int)Wire.read();
        secondHumiByte = (int)Wire.read();
        thirdHumiByte = (int)Wire.read();
      }
      float calculatedHumidity = -6 + (125 * (((firstHumiByte << 8) + secondHumiByte) / (pow(2, 16))));

      Serial.print("Hum: ");
      Serial.println(calculatedHumidity);

      /* Task2 has/had the higher priority, so for Task1 to reach here Task2
    must have already executed and deleted itself.  Delay for 100
    milliseconds. */
      vTaskDelay(xDelay200ms);
      vTaskDelete(xTaskHumHandle);
    }
  }
}

void vTaskServoAuxiliar(void *pvParameters) {
  int pos_servo;
  pinMode(servoPin, OUTPUT);     // set servo pin as “output”
  for (int i = 0; i <= 15; i++)  // giving the servo time to rotate to commanded position
  {
    servopulse(0);  //inicialização do servo para o valor atual
  }
  for (;;) {
    if (xQueueReceive(xServoQueue, &pos_servo, portMAX_DELAY) != errQUEUE_EMPTY) {
      for (int i = 0; i <= 15; i++)  // giving the servo time to rotate to commanded position
      {
        servopulse(180 * pos_servo / 4);  // use the pulse function
      }
    }
  }
}

void servopulse(int myangle)  // define a servo pulse function
{
  int pulsewidth = (myangle * 11) + 500;  // convert angle to 500-2480 pulse width
  digitalWrite(servoPin, HIGH);           // set the level of servo pin as “high”
  delayMicroseconds(pulsewidth);          // delay microsecond of pulse width
  digitalWrite(servoPin, LOW);            // set the level of servo pin as “low”
  vTaskDelay((20 - pulsewidth / 1000) / portTICK_PERIOD_MS);
}
