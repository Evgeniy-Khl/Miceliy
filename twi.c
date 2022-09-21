void read_clock(char amount)
{
  unsigned char i;
  TWCR = 0xA4;//(1<<TWINT)|(1<<TWSTA)|(TWEN) - Send START condition
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the START condition has been transmitted
  if ((TWSR & 0xF8) != START_OK) goto EXIT_read_clock;
  TWDR = 0xD0+0;//Load SLA_W into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_SLA_ACK) goto EXIT_read_clock;
  TWDR = 0;//Load DATA into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_DATA_ACK) goto EXIT_read_clock;
  TWCR = 0xA4;//(1<<TWINT)|(1<<TWSTA)|(TWEN) - Send REPEATED START condition
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the START condition has been transmitted
  if ((TWSR & 0xF8) != REPIT_START_OK) goto EXIT_read_clock;
  TWDR = 0xD0+1;//Load SLA_R into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MR_SLA_ACK) goto EXIT_read_clock;
  ptr_to_ram=clock_buffer;
  for (i=0; i<amount; i++)
  {
    TWCR = 0xC4;//(1<<TWINT) | (1<<TWEN) | (1<<TWEA) *!*!*!если нет ответа в предышем сеансе то TWINT не устанавливается
    while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been received, and ACK/NACK has been received.
    *ptr_to_ram++ = TWDR;//Read DATA into TWDR register.
    if ((TWSR & 0xF8) != MR_DATA_ACK) goto EXIT_read_clock;
  } 
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been received, and ACK/NACK has been received.
  *ptr_to_ram = TWDR;//Read DATA into TWDR register.
//  if ((TWSR & 0xF8) != MR_DATA_NOT_ACK) ERROR = 1; 
EXIT_read_clock:
  TWCR = 0x94;//(1<<TWINT)|(1<<TWEN)|(1<<TWSTO) - Transmit STOP condition
}

void write_clock(char *str)
{
  unsigned char i;
  TWCR = 0xA4;//(1<<TWINT)|(1<<TWSTA)|(TWEN) - Send START condition
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the START condition has been transmitted
  if ((TWSR & 0xF8) != START_OK) goto EXIT_write_clock;
  TWDR = 0xD0+0;//Load SLA_W into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_SLA_ACK) goto EXIT_write_clock;
  TWDR = 0;//Load DATA into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_DATA_ACK) goto EXIT_write_clock;
  for (i=0; i<CLOCK_BUFFER; i++)
  {
    TWDR = *str++;//Load DATA into TWDR register.
    TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
    while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been received, and ACK/NACK has been received.
    if ((TWSR & 0xF8) != MT_DATA_ACK) goto EXIT_write_clock;
  }
EXIT_write_clock:
  TWCR = 0x94;//(1<<TWINT)|(1<<TWEN)|(1<<TWSTO) - Transmit STOP condition 
}
/*
unsigned char read_AT24C512(unsigned int addres) // передаем адресс, возвращаем значение
{
  unsigned char data=255;
 
  TWCR = 0xA4;//(1<<TWINT)|(1<<TWSTA)|(TWEN) - Send START condition
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the START condition has been transmitted
  if ((TWSR & 0xF8) != START_OK) goto EXIT_AT24C512;
  TWDR = 0xA0;//Load SLA_W into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_SLA_ACK) goto EXIT_AT24C512;
  ptr_to_ram = &addres;
  ptr_to_ram++;
  TWDR = *ptr_to_ram--;//Load WORD HI ADDRESS into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_DATA_ACK) goto EXIT_AT24C512;
  TWDR = *ptr_to_ram;//Load WORD LOW ADDRESS into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MT_DATA_ACK) goto EXIT_AT24C512;
  TWCR = 0xA4;//(1<<TWINT)|(1<<TWSTA)|(TWEN) - Send REPEATED START condition
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the START condition has been transmitted
  if ((TWSR & 0xF8) != REPIT_START_OK) goto EXIT_AT24C512;
  TWDR = 0xA1;//Load SLA_R into TWDR register. Clear TWINT bit in TWCR to start transmission of address
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
  if ((TWSR & 0xF8) != MR_SLA_ACK) goto EXIT_AT24C512;
  TWCR = 0x84;//(1<<TWINT) | (1<<TWEN)
  while (!(TWCR & 0x80));//Wait for TWINT flag set. This indicates that the DATA has been received, and ACK/NACK has been received.
  data = TWDR;//Read DATA into TWDR register.  ЗДЕСЬ ЗАВИСАЕТ СИСТЕМА
//  if ((TWSR & 0xF8) != MR_DATA_NOT_ACK) goto EXIT_AT24C512;
EXIT_AT24C512: 
  TWCR = 0x94;//(1<<TWINT)|(1<<TWEN)|(1<<TWSTO) - Transmit STOP condition
  return data;
}
*/