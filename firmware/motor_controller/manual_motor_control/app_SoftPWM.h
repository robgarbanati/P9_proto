#define LED_PWM_PERIOD 64

#define RGB_RED			LED_PWM_PERIOD, 0, 0
#define RGB_GREEN		0, LED_PWM_PERIOD, 0
#define RGB_BLUE		0, 0, LED_PWM_PERIOD
#define RGB_YELLOW		LED_PWM_PERIOD/2, LED_PWM_PERIOD, 0
#define RGB_ORANGE		LED_PWM_PERIOD, LED_PWM_PERIOD/2, 0
#define RGB_MAGENTA		LED_PWM_PERIOD, 0, LED_PWM_PERIOD
#define RGB_CYAN		0, LED_PWM_PERIOD, LED_PWM_PERIOD
#define RGB_WHITE		LED_PWM_PERIOD, LED_PWM_PERIOD, LED_PWM_PERIOD
#define RGB_PINK		LED_PWM_PERIOD, LED_PWM_PERIOD/10, LED_PWM_PERIOD/10
#define RGB_OFF			0, 0, 0

void RGB_init(void);
void RGB_handle(void);
void RGB_set(uint8_t R, uint8_t G, uint8_t B);
