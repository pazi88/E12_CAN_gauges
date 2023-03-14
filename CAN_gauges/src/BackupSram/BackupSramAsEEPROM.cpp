#include "BackupSramAsEEPROM.h"


    BackupSramAsEEPROM::BackupSramAsEEPROM(){
          //Enable the power interface clock
          RCC->AHBENR |= RCC_APB1ENR_PWREN;
        
          //Enable the backup SRAM clock by setting BKPSRAMEN bit i
          RCC->AHBENR |= RCC_AHBENR_SRAMEN;

          /** If the HSE divided by 2, 3, ..31 is used as the RTC clock, the
            * Backup Domain Access should be kept enabled. */

          // Enable access to Backup domain
          PWR->CR |= PWR_CR_DBP;

          /** enable the backup regulator (used to maintain the backup SRAM content in
            * standby and Vbat modes).  NOTE : this bit is not reset when the device
            * wakes up from standby, system reset or power reset. You can check that
            * the backup regulator is ready on PWR->CSR.brr, see rm p144 */

          //Enable the backup power regulator. This makes the sram backup possible. bit is not reset by software!
          PWR->CSR |= PWR_CSR_SBF;

          //Wait until the backup power regulator is ready
          while ((PWR->CSR & PWR_CSR_SBF) == 0);
    }
    uint16_t BackupSramAsEEPROM::length(){ return 4096; }
    int8_t BackupSramAsEEPROM::write_byte( uint8_t *data, uint16_t bytes, uint16_t offset ) {
        uint8_t* base_addr = (uint8_t *) SRAM_BASE;
        uint16_t i;
          if( bytes + offset >= backup_size ) {
            /* ERROR : the last byte is outside the backup SRAM region */
            return -1;
          }
        
          /* disable backup domain write protection */
          //Set the Disable Backup Domain write protection (DBP) bit in PWR power control register
          //PWR->CR |= PWR_CR_DBP;

          for( i = 0; i < bytes; i++ ) {
            *(base_addr + offset + i) = *(data + i);
          }
          //Enable write protection backup sram when finished  
          //PWR->CR &= ~PWR_CR_DBP;
          return 0;
        }
        
    int8_t BackupSramAsEEPROM::read_byte( uint8_t *data, uint16_t bytes, uint16_t offset ) {
          uint8_t* base_addr = (uint8_t *) SRAM_BASE;
          uint16_t i;
          if( bytes + offset >= backup_size ) {
            /* ERROR : the last byte is outside the backup SRAM region */
            return -1;
          }
          
          for( i = 0; i < bytes; i++ ) {
            *(data + i) = *(base_addr + offset + i);
          }
          return 0;
        }
  
    uint8_t BackupSramAsEEPROM::read(uint16_t address) {
        uint8_t val = 0;
        read_byte(&val, 1, address);
      
        return val;
    }

    uint32_t BackupSramAsEEPROM::read32(uint16_t address) {
        uint32_t four = 0;
        uint32_t three = 0;
        uint32_t two = 0;
        uint32_t one = 0;
        four = read( address );
        three = read( address + 1 );
        two = read( address + 2 );
        one = read( address + 3 );
        
        return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
    }
    
    int8_t BackupSramAsEEPROM::write(uint16_t address, uint8_t val) {
        write_byte(&val, 1, address);   
        return 0;
    }
	
    int8_t BackupSramAsEEPROM::write32(uint16_t address, uint32_t val) {
        uint8_t four = (val & 0xFF);
        uint8_t three = ((val >> 8) & 0xFF);
        uint8_t two = ((val >> 16) & 0xFF);
        uint8_t one = ((val >> 24) & 0xFF);
		write_byte(&four, 1, address);
		write_byte(&three, 1, address + 1);
		write_byte(&two, 1, address + 2);
		write_byte(&one, 1, address + 3);
        return 0;
    }
    
    int8_t BackupSramAsEEPROM::update(uint16_t address, uint8_t val) {
        write_byte(&val, 1, address);   
        return 0;
    }
