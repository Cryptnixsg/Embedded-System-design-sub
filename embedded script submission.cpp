#include "mbed.h"
#include "stdbool.h"
#include "PinNames.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

// Define macros for motor control pins
#define MOTOR_ENA_Pin   GPIO_PIN_9    // Replace '9' with the actual pin number for MOTOR_ENA (PA9)
#define MOTOR_ENA_GPIO_Port GPIOA      // The ENA pin is connected to Port A

#define MOTOR_ENB_Pin   GPIO_PIN_3    // Replace '3' with the actual pin number for MOTOR_ENB (PD3)
#define MOTOR_ENB_GPIO_Port GPIOD      // The ENB pin is connected to Port D

#define MOTOR_IN1_Pin   GPIO_PIN_8    // Replace '8' with the actual pin number for MOTOR_IN1 (PB8)
#define MOTOR_IN1_GPIO_Port GPIOB      // The IN1 pin is connected to Port B

#define MOTOR_IN2_Pin   GPIO_PIN_7    // Replace '7' with the actual pin number for MOTOR_IN2 (PB7)
#define MOTOR_IN2_GPIO_Port GPIOB      // The IN2 pin is connected to Port B

#define MOTOR_IN3_Pin   GPIO_PIN_5    // Replace '5' with the actual pin number for MOTOR_IN3 (PB5)
#define MOTOR_IN3_GPIO_Port GPIOB      // The IN3 pin is connected to Port B

#define MOTOR_IN4_Pin   GPIO_PIN_4    // Replace '4' with the actual pin number for MOTOR_IN4 (PB4)
#define MOTOR_IN4_GPIO_Port GPIOB      // The IN4 pin is connected to Port B


// Define macros for LED control pins
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA


// Define macros for proximity sensor pins
#define PROXIMITY_SENSOR_Pin PC_1

// Define macros for ultrasonic sensor pins
#define ULTRASONIC_TRIGGER_Pin GPIO_PIN_0
#define ULTRASONIC_TRIGGER_GPIO_Port GPIOA
#define ULTRASONIC_ECHO_Pin GPIO_PIN_5
#define ULTRASONIC_ECHO_GPIO_Port GPIOA

// Define macros for Bluetooth UART communication
#define BT_UART_Handle huart2 // Update this with your actual UART handle

// Define macros for manual override pin or button
#define MANUAL_OVERRIDE_Pin PA_1

// Define macros for the directional control buttons
#define FORWARD_BUTTON_Pin  PB_10
#define BACKWARD_BUTTON_Pin PB_11
#define LEFT_BUTTON_Pin     PB_9
#define RIGHT_BUTTON_Pin    PB_8

// New Bluetooth command characters for manual override
#define COMMAND_ENABLE_MANUAL_OVERRIDE 'M'
#define COMMAND_DISABLE_MANUAL_OVERRIDE 'N'

// Defining button pin 
#define BUTTON_Pin_Port  GPIOA
#define BUTTON_Pin       GPIO_PIN_0



// Function prototypes
void Motor_Init(void);
void LED_Init(void);
void ProximitySensor_Init(void);
void Ultrasonic_Init(void); // Implement this function
void Bluetooth_Init(void);

void Level1_PushButtonNavigation(void);
void Level2_ProximityAutoNavigation(void);
void Level3_RemoteBTNavigation(void);
void Level4_AutoUltrasonicNavigation(void);
bool IsObstacleDetected(void); // Implement this function
void Bluetooth_ReceiveCommand(void); // Implement this function
bool IsmanualOverrideEnabled(void); // Implement this function

// Define the interrupt flag to indicate if a Bluetooth command interrupt occurred
volatile bool bluetoothCommandInterrupt = false;

// Bluetooth command variable to store the received command
volatile char bluetoothCommand = '\0';

// UART receive buffer
volatile char uartReceiveBuffer[1];
UART_HandleTypeDef BT_UART_Handle;

//declare manual override as a global variable
volatile bool manualOverrideEnabled = false;



// UART receive complete callback function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &BT_UART_Handle) {
        // Process the received Bluetooth command here
        bluetoothCommand = uartReceiveBuffer[0];
        if (bluetoothCommand == 'O') {
            // Enable manual override
            manualOverrideEnabled = true;
        } else if (bluetoothCommand == 'C') {
            // Disable manual override
            manualOverrideEnabled = false;
        } else {
            // Process other commands if needed
        }
        bluetoothCommandInterrupt = true;
    }
}

volatile float ultrasonicDistance = 0.0;

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


void Motor_Init(void) {
    // Initialize motor control GPIO pins as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MOTOR_ENA_Pin | MOTOR_IN1_Pin | MOTOR_IN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PWM for motor speed control
TIM_HandleTypeDef htim;
TIM_OC_InitTypeDef sConfig;

// Replace TIMx with the appropriate timer instance (e.g., TIM1, TIM2, TIM3, etc.)
htim.Instance = TIM2;

// Set the PWM period (time for one PWM cycle)
// The value can be adjusted based on the desired motor speed range and motor specifications
htim.Init.Period = 1000; // For example, set to 1000 for 1 kHz PWM frequency

// Start the PWM timer
HAL_TIM_PWM_Init(&htim);

// Set the PWM pulse width (duty cycle)
sConfig.OCMode = TIM_OCMODE_PWM1;
sConfig.Pulse = 500; // For example, set to 500 for 50% duty cycle (50% speed)
sConfig.OCPolarity = TIM_OCPOLARITY_HIGH; // You can change this polarity based on your motor driver

HAL_TIM_PWM_ConfigChannel(&htim, &sConfig, TIM_CHANNEL_1); // Replace TIM_CHANNEL_x with the appropriate channel (e.g., TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)

HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
}

void Ultrasonic_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure the trigger pin as output
    GPIO_InitStruct.Pin = ULTRASONIC_TRIGGER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ULTRASONIC_TRIGGER_GPIO_Port, &GPIO_InitStruct);

    // Configure the echo pin as input
    GPIO_InitStruct.Pin = ULTRASONIC_ECHO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ULTRASONIC_ECHO_GPIO_Port, &GPIO_InitStruct);
}

void ProximitySensor_Init(void) {
    // Implementation can be added in the future if proximity sensor is needed
}

void Motor_Forward(void) {
    // Set the direction pins for forward motion
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);

    // Adjust PWM duty cycle for desired motor speed
    TIM_HandleTypeDef htim;
    htim.Instance = TIM2; // Replace TIMx with the appropriate timer instance
    htim.Instance->CCR1 = 500; // For example, set to 500 for 50% duty cycle (50% speed)
}

void Motor_MoveRight(void) {
    // Set the direction pins for turning right
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);

    // Adjust PWM duty cycle for desired motor speed (if needed)
    TIM_HandleTypeDef htim;
    htim.Instance = TIM2;
    htim.Instance->CCR1 = 300; // For example, set to 300 for 30% duty cycle (30% speed)
}

void Motor_MoveLeft(void) {
    // Set the direction pins for turning left
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);

    // Adjust PWM duty cycle for desired motor speed (if needed)
    TIM_HandleTypeDef htim;
    htim.Instance = TIM2;
    htim.Instance->CCR1 = 300; // For example, set to 300 for 30% duty cycle (30% speed)
}

void Motor_Backward(void) {
    // Set the direction pins for backward motion
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);

    // Adjust PWM duty cycle for desired motor speed
    TIM_HandleTypeDef htim;
    htim.Instance = TIM2; // Replace TIMx with the appropriate timer instance
    htim.Instance->CCR1 = 500; // For example, set to 500 for 50% duty cycle (50% speed)
}


void Motor_Stop(void) {
    // Stop the PWM output by setting the duty cycle to 0
    TIM_HandleTypeDef htim;
    htim.Instance = TIM2; // Replace TIMx with the appropriate timer instance
    htim.Instance->CCR1 = 0;

    // Disable the motor driver by setting the ENA pin to LOW
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_RESET);

    // Set the direction pins to LOW to ensure the motor is not moving
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
}

// Function to get the distance measured by the HC-SR04 ultrasonic sensor
float Ultrasonic_GetDistance(void) {
    // Send a trigger pulse to start the ultrasonic measurement
    HAL_GPIO_WritePin(ULTRASONIC_TRIGGER_GPIO_Port, ULTRASONIC_TRIGGER_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Wait for a short duration (e.g., 10 microseconds)
    HAL_GPIO_WritePin(ULTRASONIC_TRIGGER_GPIO_Port, ULTRASONIC_TRIGGER_Pin, GPIO_PIN_RESET);

    // Measure the duration of the echo pulse
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET) {
        startTime = HAL_GetTick();
    }
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET) {
        endTime = HAL_GetTick();
    }

    // Calculate the distance based on the duration of the echo pulse
    float pulseDuration = (float)(endTime - startTime);
    // Speed of sound in air is approximately 343 meters per second (34300 centimeters per second)
    // Divide by 2 to account for the round trip of the ultrasonic signal
    float distance = pulseDuration * 34300.0 / 2.0 / 1000.0; // Convert to centimeters

    return distance;
}


// Level 1 PushButtonNavigation
void Level1_PushButtonNavigation(void) {
    // Implement the Level 1 push-button navigation logic here
    while (1) {
        // Check for Bluetooth command interrupt
        if (bluetoothCommandInterrupt) {
            // Process the Bluetooth command here
            switch (bluetoothCommand) {
                case 'W': // 'W' key corresponds to forward
                    Motor_Forward();
                    break;
                case 'S': // 'S' key corresponds to backward
                    Motor_Backward();
                    break;
                case 'A': // 'A' key corresponds to left
                    // Implement left turn logic (e.g., turn left for a specific duration)
                    Motor_Stop();
                    Motor_Backward(); // Move backward to adjust the position
                    HAL_Delay(1000);  // Adjust the delay for the required turning duration
                    Motor_Stop();     // Stop the movement after turning
                    break;
                case 'D': // 'D' key corresponds to right
                    // Implement right turn logic (e.g., turn right for a specific duration)
                    Motor_Stop();
                    Motor_Backward(); // Move backward to adjust the position
                    HAL_Delay(1000);  // Adjust the delay for the required turning duration
                    Motor_Stop();     // Stop the movement after turning
                    break;
                case 'X': // 'X' key corresponds to stop (emergency stop or immediate stop)
                    Motor_Stop();
                    break;
                // Add more cases for other commands as needed
                default:
                    // Invalid command received, do nothing or handle the error
                    break;
            }

            // Reset the Bluetooth command interrupt flag
            bluetoothCommandInterrupt = false;
        } else {
            // No Bluetooth command received, stop the motors
            Motor_Stop();
        }
    }
}

// Level 2 ProximityAutoNavigation
void Level2_ProximityAutoNavigation(void) {
    // Implement the Level 2 proximity-based auto navigation logic here
    while (1) {
        // Check for manual override
        if (IsmanualOverrideEnabled()) {
        // Read the distance from the ultrasonic sensor
        float distance = Ultrasonic_GetDistance();

        // Print the distance value
        printf("Distance: %.2f cm\r\n", distance);

        // Check for Bluetooth command interrupt
        if (bluetoothCommandInterrupt) {
            // Process the Bluetooth command here
            switch (bluetoothCommand) {
                case 'W':
                    Motor_Forward();
                    HAL_Delay(1000);
                    break;
                case 'S':
                    Motor_Backward();
                    HAL_Delay(1000);
                    break;
                case 'A':
                    Motor_MoveLeft();
                    HAL_Delay(2000);
                    break;
                case 'D':
                    Motor_MoveRight();
                    HAL_Delay(2000);
                    break;
                default:
                    Motor_Stop(); // Stop for any other command or invalid command
                    break;
            }

            // Reset the Bluetooth command interrupt flag
            bluetoothCommandInterrupt = false;
        } else {
            Motor_Stop(); // Stop if no command is received
        }

        // Check for obstacles using the ultrasonic sensor
        if (distance < 10.0) {
            // If an obstacle is detected within 10 cm, stop and turn to avoid it
            Motor_Stop();

            // Implement obstacle avoidance logic (randomly choose left or right)
            if (rand() % 2 == 0) {
                // Turn left
                Motor_MoveLeft();
            } else {
                // Turn right
                Motor_MoveRight();
            }

            // After avoiding the obstacle, you can continue moving forward or implement other behaviors.
            Motor_Forward(); // For example, continue moving forward after avoiding the obstacle
            HAL_Delay(1000); // Adjust the delay based on your requirements
        }

        // Add some delay to prevent continuous reading and movement (adjust the delay as needed)
        HAL_Delay(500);
    }
}


}



// Level 3 RemoteBTNavigation
void Level3_RemoteBTNavigation(){
    // Implement the Level 3 remote Bluetooth-based navigation logic here
    while (1) {
        // Check for Bluetooth command interrupt
        if (IsmanualOverrideEnabled()) {
            // Process the Bluetooth command here
            switch (bluetoothCommand) {
                case 'F':
                    Motor_Forward();
                    break;
                case 'B':
                    Motor_Backward();
                    break;
                case 'L':
                    // Implement left turn logic (e.g., turn left for a specific duration)
                    Motor_Stop();
                    Motor_MoveLeft(); // Turn left according to input
                    HAL_Delay(1000);  // Adjust the delay for the required turning duration
                    Motor_Stop();     // Stop the movement after turning
                    break;
                case 'R':
                    // Implement right turn logic (e.g., turn right for a specific duration)
                    Motor_Stop();
                    Motor_MoveRight(); // Turn right according to input
                    HAL_Delay(1000);  // Adjust the delay for the required turning duration
                    Motor_Stop();     // Stop the movement after turning
                    break;
                case 'S':
                    Motor_Stop();
                    break;
                // Add more cases for other commands as needed
                default:
                    // Invalid command received, do nothing or handle the error
                    break;
            }

            // Reset the Bluetooth command interrupt flag
            bluetoothCommandInterrupt = false;
        } else {
            // Continue with auto-navigation logic
            Motor_Forward(); // Move forward if no obstacle detected

            // Read the distance from the ultrasonic sensor
            float distance = Ultrasonic_GetDistance();

            // Print the distance value
            printf("Distance: %.2f cm\r\n", distance);

            // Check for obstacles using the ultrasonic sensor
            if (distance < 10.0) {
                // If an obstacle is detected within 10 cm, stop and turn to avoid it
                Motor_Stop();

                // Implement obstacle avoidance logic (e.g., turn left or right)

                // For example, to turn left:
                Motor_Backward(); // Move backward to adjust the position
                HAL_Delay(500);  // Adjust the delay for the required reversing duration
                Motor_MoveLeft(); // Turn left to adjust the position
                HAL_Delay(1000);  //Adjust the delay for the required turning duration
                Motor_Stop();     // Stop the movement after turning

                Motor_Forward(); // For example, continue moving forward after avoiding the obstacle
                HAL_Delay(1000); // Adjust the delay based on your requirements

            }

            // Add some delay to prevent continuous reading and movement (adjust the delay as needed)
            HAL_Delay(500);
        }
    }
}
// Initialize the HC-05 Bluetooth module
void Bluetooth_Init(void) {
    // Enable the UART peripheral clock
    __HAL_RCC_USART1_CLK_ENABLE();

    // Configure the UART pins (TX and RX pins)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10; // PA9 (TX) and PA10 (RX)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure the UART peripheral
    BT_UART_Handle.Instance = USART1;
    BT_UART_Handle.Init.BaudRate = 9600; // Set the desired baud rate (e.g., 9600 bps)
    BT_UART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
    BT_UART_Handle.Init.StopBits = UART_STOPBITS_1;
    BT_UART_Handle.Init.Parity = UART_PARITY_NONE;
    BT_UART_Handle.Init.Mode = UART_MODE_TX_RX;
    BT_UART_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    BT_UART_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    
}

// Check if an obstacle is detected by the ultrasonic sensor
bool IsObstacleDetected(void) {
    // Implement the logic to check if an obstacle is detected using the ultrasonic sensor
    // For example, call the Ultrasonic_GetDistance function and check if the distance is below a threshold
    float distance = Ultrasonic_GetDistance();
    return distance < 10.0; // Return true if an obstacle is detected within 10 cm, otherwise false
}

// Check if manual override is enabled (you can use any hardware input, e.g., a button)
bool IsManualOverrideEnabled(void) {
    // ... (previous code)
    return manualOverrideEnabled;
}

// Receive and process commands from the HC-05 Bluetooth module
void Bluetooth_ReceiveCommand(void) {
    // Check if there is any data available in the UART receive buffer
    if (BT_UART_Handle.RxState == HAL_UART_STATE_READY) {
        // Receive the data from UART buffer
        if (HAL_UART_Receive_IT(&BT_UART_Handle, (uint8_t *)uartReceiveBuffer, 1) == HAL_OK) {
            // Process the received data, e.g., store it in a global variable
            // In this example, the received data is expected to be a single character representing the command
            bluetoothCommand = uartReceiveBuffer[0];
            bluetoothCommandInterrupt = true;

            // Process the received Bluetooth command using a switch statement
            switch (bluetoothCommand) {
                case 'F': // Move forward command received
                    Motor_Forward();
                    break;
                case 'B': // Move backward command received
                    Motor_Backward();
                    break;
                case 'L': // Turn left command received
                    Motor_MoveLeft();
                    break;
                case 'R': // Turn right command received
                    Motor_MoveRight();
                    break;
                case 'S': // Stop command received
                    Motor_Stop();
                    break;
                case 'M': // Enable manual override command received
                    // Set the manualOverrideEnabled variable to true
                    manualOverrideEnabled = true;
                    break;
                case 'N': // Disable manual override command received
                    // Set the manualOverrideEnabled variable to false
                    manualOverrideEnabled = false;
                    break;
                // Add more cases for other commands as needed
                default:
                    // Invalid command received, do nothing or handle the error
                    break;
            }
        }
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();

    // The voltage scaling allows optimizing the power consumption when the device is
    // clocked below the maximum system frequency, to update the voltage scaling value
    // regarding system frequency refer to product datasheet
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;


    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
}   



int main(void) {
    // MCU Initialization
    HAL_Init();
    SystemClock_Config();

    // Peripheral Initializations
    Motor_Init();
    Ultrasonic_Init(); // Initialize the ultrasonic sensor
    LED_Init();
    ProximitySensor_Init();
    Bluetooth_Init();

    // Enable UART receive interrupt for Bluetooth commands
    HAL_UART_Receive_IT(&BT_UART_Handle, (uint8_t *)uartReceiveBuffer, 1);

    // Define variables to keep track of button press and LED state
    int buttonPressCount = 0;
    bool ledOn = false;

    while (1) {
    // Check for button press
    if (HAL_GPIO_ReadPin(BUTTON_Pin_Port, BUTTON_Pin) == GPIO_PIN_RESET) {
        // Button is pressed, wait for button release
        while (HAL_GPIO_ReadPin(BUTTON_Pin_Port, BUTTON_Pin) == GPIO_PIN_RESET) {
            // Add a short delay for button debouncing
            HAL_Delay(50);
        }

        // Button is released, increment buttonPressCount
        buttonPressCount++;

        // Ensure motors are fully stopped before switching levels
        Motor_Stop();
        HAL_Delay(500); // Small delay to ensure motor stop is complete

        // Check for any remaining commands and process them
        if (bluetoothCommandInterrupt) {
            Bluetooth_ReceiveCommand(); // Process remaining Bluetooth commands
            bluetoothCommandInterrupt = false; // Reset interrupt flag
        }

        // Switch to the appropriate navigation level
        if (buttonPressCount == 1) {
            ledOn = false; // Level 1, turn off the LED
            Level1_PushButtonNavigation();
        } else if (buttonPressCount == 2) {
            ledOn = true; // Level 2, turn on the LED
            Level2_ProximityAutoNavigation();
        } else if (buttonPressCount == 3) {
            ledOn = true; // Level 3, turn on the LED
            Level3_RemoteBTNavigation();
        }

        // Reset buttonPressCount if it exceeds 3 to cycle back to Level 1
        if (buttonPressCount > 3) {
            buttonPressCount = 1;
        }

        // Control the LED based on the current level
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ledOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}

