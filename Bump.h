

/**
 * Initialize Bump sensors<br>
 * Make Port 4 pins 7,6,5,3,2,0 as inputs<br>
 * Activate interface pullup
 * @param none
 * @return none
 * @brief  Initialize Bump sensors
 */
void EdgeTrigger_Init(void);

/**
 * Read current state of 6 bump switches<br>
 * Read Port 4 pins 7,6,5,3,2,0 inputs<br>
 * Returns a 6-bit positive logic result (0 to 63)<br>
 * bit 5 Bump5<br>
 * bit 4 Bump4<br>
 * bit 3 Bump3<br>
 * bit 2 Bump2<br>
 * bit 1 Bump1<br>
 * bit 0 Bump0
 * @param none
 * @return result is 6-bit positive logic
 * @note  result is a packed, right-justified, positive logic
 * @brief  Read current state of 6 switches
 */
uint8_t Bump_Read(void);
