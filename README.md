# HL_Filters
A transmit and receive filter set for a Hermes-Lite with microprocessor controlled switching,

The microprocessor in this switchable filter responds to inputs from an HL1 where it expects a command to switch bands 0 .. 7 and a hardware MOX signal or alternatively an HL2 8 bit byte from the I2C bus where it will switch TX and RX filters plus MOX according to ther byte coding. To choose between HL1 or HL2 the state of the I2C bus is detected. If using HL1 a jumper plug is installed on the I2C socket which pulls SDA and SCL low. 

The main loop is polled for the state of SDA and SCL and if both LOW looks for a change of filter state and it is switched if changed. The MOX state is then checked and the appropriate filter switching implemented.
