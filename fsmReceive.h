#ifndef FSM_H_
#define FSM_H_

void initialState(void);
void readByte(void);
void preambleByte(void);
void lengthByte(void);
void packetTypeByte(void);
void message(void);
void crcCheck(void);
void storeValues(void);
void fsmReceive(void);
uint8_t checkModeByte(uint8_t byte);

#endif