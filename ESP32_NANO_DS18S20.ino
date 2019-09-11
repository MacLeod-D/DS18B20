// Here for ESP32, but should work with Nano/Uno without any changes



/*

 There are a lot of programs around to read temperatures from 1 wire DS1820, DS18B20, DS18S20 devices

 Why another one ?

 1. This program does NOT use any library !
    Libraries are good to get simple access to devices. But they hide what is going on behind the scene. And sometimes
    you want to understand what they are doing.
    Normally Wire- and Dallas-libraries are used.

 2. This program searches for connected one-wire slaves and uses all found DS18(B)20-devices.
    Perfect for measuring temperatures in different roms.
    It will run even if one device breaks !

 3. Normally temperatures are changing slowly compared to the speed of measurements.
    But you always get noise at least of +/- 1 bit.   
    This program shows a simple but efficient method to filter the measured values to get smooth curves and rise the overall resolution.

 3. Most libraries just work with fixed delays (depending on resolution) internaly.
    Here we call a state machine function [ DoJob2() ] while waiting for measurements to complete (about 600 ms for 12 bit resolution!)

    DoJob2() is called about 16000 times per second !
    It is devided in 4 parts - it is possible to call 4 Functions in a round robin fashion.
    It is garanteed, that the maximum delay from one call to another is less than 400 µs.

    Using this simple multitasking most of the CPU time may be used for other jobs than measuring temperatures.

 4. 

    Enjoy it.
    
*/




//https://wiki.arduino-hannover.de/wiki/DS1820_Temperatursensor_1-Wire
//https://pic-projekte.de/blog/ds18s20/


// OneWire DS18S20, DS18B20, DS1822 Temperature Example
// on signal pin (a single 4.7K resistor is necessary)                    >>>>>>>> I needed 3.3k !!!!!!!!!!!!!!!!

#include "Config.h"


byte rom_c[DEVNUM][8];
bool type_s[DEVNUM];
double cel[DEVNUM];
float celsius[DEVNUM], fahrenheit[DEVNUM];
int devices=0;
bool Out=false;

unsigned long Job2Cnt, Part0, Part1, Part2, Part3, MaxWait;

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
// #define HIGH_ALARM_TEMP 2
// #define LOW_ALARM_TEMP  3
// #define CONFIGURATION   4
// #define INTERNAL_BYTE   5
// #define COUNT_REMAIN    6
// #define COUNT_PER_C     7
// #define SCRATCHPAD_CRC  8

//----------------------------------------------- forward declaratio ---------------------------------
void DoJob2();


//----------------------------------------------- One wire functions ---------------------------------


/*******************************************************************************
One_Wire_Master-Reset-Impuls
--------------------------------------------------------------------------------
This function will release the master reset impuls and checks if there is at
least one 1-wire-component connected to the bus (0) or not (1).
*******************************************************************************/
unsigned char ow_reset() {
unsigned char ret;
  pinMode(OneWirePin,OUTPUT);
  digitalWrite(OneWirePin,0);
  delayMicroseconds(500);
  pinMode(OneWirePin,INPUT);
  delayMicroseconds(50);
  ret=digitalRead(OneWirePin);
  delayMicroseconds(450);
  return(ret);
}


/*******************************************************************************
 One_Wire_Write_Bit
 -------------------------------------------------------------------------------
 This function will write one single bit on the bus.
 *******************************************************************************/
void ow_wr_bit (uint8_t val)
{   
    digitalWrite(OneWirePin,0);     // set the (I)O to low level       
    pinMode(OneWirePin,OUTPUT);     // config the DQ-IO as output (-> low)
    if(val)                         // if the bit to transfer is a "1"
    {
        delayMicroseconds(1);       // wait 1 us and..             
        pinMode(OneWirePin,INPUT);  // ..config the DQ-IO as input (high-z -> pull up)
    }
    delayMicroseconds(100);         // wait for end of slot         
    DoJob2();
    pinMode(OneWirePin,INPUT);      // config the DQ-IO as input (high-z -> pull up)
 }
 
/*******************************************************************************
 One_Wire_Write_Byte
 -------------------------------------------------------------------------------
 This function will write a complete byte on the bus.
 *******************************************************************************/
void ow_wr_byte (uint8_t val)
{
    uint8_t i, mask = 1;
    // write the byte by sending eight bits (LSB first)
    for (i=0; i<8; i++)
    {
        ow_wr_bit(val & mask);
        mask = (mask << 1);
        DoJob2();
    }
}


/*******************************************************************************
 One_Wire_Read_Bit
 -------------------------------------------------------------------------------
 This function will read one single bit from the bus.
 *******************************************************************************/
uint8_t ow_rd_bit (void)
{
    uint8_t rec;                      // perform a very short low impuls
                                      // config the DQ-IO as output (-> low)
    pinMode(OneWirePin,OUTPUT);    
    pinMode(OneWirePin,INPUT);        // config the DQ-IO as input (high-z -> pull up)
    delayMicroseconds(15);             
    rec=digitalRead(OneWirePin);      // read the level on DQ (this is the read bit)
    DoJob2();
    delayMicroseconds(105);           // wait for end of slot
    return(rec);
}


/*******************************************************************************
 One_Wire_Read_Byte
 --------------------------------------------------------------------------------
 This function will read a complete byte from the bus.
 *******************************************************************************/
uint8_t ow_rd_byte (void)
{
    uint8_t value = 0 , i;
    // read the byte by reading eight bits (LSB first)
    for(i=0; i<8; i++)
    {
        if ( ow_rd_bit() )
        {
            value |= 0x01 << i;
        }
        DoJob2();
    }
    return(value);
}

/*******************************************************************************
 One_Wire_Master_Reset
 --------------------------------------------------------------------------------
 This function will reset the bus.
 *******************************************************************************/
bool ow_mri (void)                 // Master-Reset
{
uint8_t rec;  
    digitalWrite(OneWirePin,0);       // set the (I)O to low level                
    pinMode(OneWirePin,OUTPUT);       // config the DQ-IO as output (-> low)
    DoJob2();
    delayMicroseconds(240);           // delay of >480 us       
    DoJob2();
    delayMicroseconds(250);           // delay of >480 us    
    DoJob2();            
    pinMode(OneWirePin,INPUT);        // config the  DQ-IO as input (high-z -> pull up)
    delayMicroseconds(40);                    
    rec=digitalRead(OneWirePin);      // read the level (if low, slave available)
    DoJob2();
    delayMicroseconds(200);           // wait for end of slot
    DoJob2();
    delayMicroseconds(250);           // wait for end of slot
    return (rec==0);
}



/*** Reads 64-Bit-ROM-Identifiers ***/
void ow_rd_rom(void)
{
 unsigned char i;
   //Serial.println("Read ROM");
   if(!ow_mri()) {                  //Slave presence
   {
     printf("\n\n\n\n Error: No 'Presence' vom Slave !");
     printf("\n\n Press RESET ... ");
     while(1);
     }
   }
   ow_wr_byte(0x33);                //"READ ROM" = 0x33
 
   for (i=0; i<8; i++) {            // read ID
     rom_c[devices][i] = ow_rd_byte();
     DoJob2();
   }
} 



static const uint8_t PROGMEM dscrc2x16_table[] = {
  0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
  0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
  0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
  0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};



// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table


// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
uint8_t crc = 0;
  while (len--) {
    DoJob2();
    crc = *addr++ ^ crc;  // just re-using crc as intermediate
    crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
    pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
  }
  return crc;
}


bool ow_isConversionComplete(void)
{
  return (ow_rd_bit() == 1);
}  



    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;


void ow_reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for (int j=0; j<DEVNUM; j++) {
    for(int i = 7; ; i--) {
      rom_c[j][i] = 0;
      if ( i == 0) break;
    }
  }
}




// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool ow_search(uint8_t *newAddr, bool search_mode /* = true */)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!ow_mri()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
        ow_wr_byte(0xF0);   // NORMAL SEARCH
      } else {
        ow_wr_byte(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = ow_rd_bit();
         cmp_id_bit = ow_rd_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((rom_c[devices][rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              rom_c[devices][rom_byte_number] |= rom_byte_mask;
            else
              rom_c[devices][rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            ow_wr_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !rom_c[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = rom_c[devices][i];
   }
   return search_result;
}





//----------------------------------------------- setup ----------------------------------------------

void setup(void) {
uint8_t address[9];
bool found;

  pinMode(5,OUTPUT);
  Serial.begin(921600);
  delay(1000);
  #ifdef DEBUG 
  Serial.println("\nSearch devices ...");
  #endif

  ow_reset_search();
  DoJob2();
    
  while(1) {
    found=ow_search(address, true);
    DoJob2();
    if (found) {

      
#ifdef DEBUG 
      Serial.print("  FOUND: ");
      DoJob2();
           
      Serial.print("ROM =");
      DoJob2();
      
      for(int i = 0; i < 8; i++) {
        DoJob2();
    
        Serial.write(' ');
        Serial.print(rom_c[devices][i], HEX);
      }
      Serial.print(" ");
#endif

      // the first ROM byte indicates which chip
      switch (rom_c[devices][0]) {
        case 0x10:
          #ifdef DEBUG 
          Serial.println("  Chip = DS18S20  or older DS1820");  // or old DS1820
          #endif
          type_s[devices] = true;
          break;
        case 0x28:
          #ifdef DEBUG 
          Serial.println("  Chip = DS18B20");
          #endif
          type_s[devices] = false;
          break;
        case 0x22:
          #ifdef DEBUG 
          Serial.println("  Chip = DS1822");
          #endif
          type_s[devices] = false;
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;
      }
      DoJob2();
      devices++;




      
    }
    else {
      #ifdef DEBUG 
      Serial.print(devices); Serial.println(" devices found\n");
      DoJob2();
    
      #endif
      break;
    }
  }
    

 



// Without SEARCH, only one device:  
//  ow_rd_rom();
//  Serial.print("ROM =");
//  for(int i = 0; i < 8; i++) {
//    Serial.write(' ');
//    Serial.print(rom_c[devices][i], HEX);
//  }
//  Serial.println();


  
//
//  // the first ROM byte indicates which chip
//  switch (rom_c[devices][0]) {
//    case 0x10:
//      #ifdef DEBUG 
//      Serial.println("  Chip = DS18S20  or older DS1820");  // or old DS1820
//      #endif
//      type_s = true;
//      break;
//    case 0x28:
//      #ifdef DEBUG 
//      Serial.println("  Chip = DS18B20");
//      #endif
//      type_s = false;
//      break;
//    case 0x22:
//      #ifdef DEBUG 
//      Serial.println("  Chip = DS1822");
//      #endif
//      type_s = false;
//      break;
//    default:
//      Serial.println("Device is not a DS18x20 family device.");
//      return;
//  }

  #ifdef DEBUG 
  Serial.println("\n");
  #endif
  MaxWait=0;
  
}



//----------------------------------------------- loop -----------------------------------------------

void loop(void) {
  byte i;
  byte present = 0;

  byte data[12];
 
  
    
  for (int dev=0; dev<devices; dev++) {   
      //if (ow_mri()) Serial.println("Slave available"); //Master Reset Impuls
      digitalWrite(5,LOW);
      DoJob2();
      
      ow_mri();
      DoJob2();
      
      ow_wr_byte(0x55);                       // Match ROM-Befehl
          
      for (i=0; i<8; i++) {                   // write ID
           ow_wr_byte(rom_c[dev][i]); 
      }
    
     // OR
     //ow_wr_byte(0xcc);                      // skip ROM
    
      unsigned long start=millis();
      ow_wr_byte(0x44);                       // start measure temp
      digitalWrite(5,HIGH);
      
      while(!ow_isConversionComplete()) {      // about 600 ms
        DoJob2();
      }
      
      #ifdef DEBUG 
      Serial.print("Dev:"); Serial.print(dev); Serial.print("  Conversion time(ms): "); Serial.print(millis()-start);
      DoJob2();
      Serial.print("  Job2-Count: "); Serial.print(Job2Cnt);
      DoJob2();
      Serial.print("  Part0: "); Serial.print(Part0);
      DoJob2();
      Serial.print("  Part1: "); Serial.print(Part1);
      DoJob2();
      Serial.print("  Part2: "); Serial.print(Part2);
      DoJob2();
      Serial.print("  Part3: "); Serial.print(Part3); Part3=0;
      DoJob2();
      Serial.print("  MaxWait: "); Serial.println(MaxWait);
      DoJob2();
      
      #endif
      
      
      
      if (!ow_mri()) {
        Serial.println("Slave not available"); //Master Reset Impuls
        delay(1000);
        return;
      }
        
          // Match ROM-Befehl aussenden
          ow_wr_byte(0x55);
         // Send ROM identifier of slave device to address it (8 Bytes)
         for (i=0; i<8; i++) {              // write ID
           ow_wr_byte(rom_c[dev][i]); 
           DoJob2();
         }
    
     // OR:
     
     //ow_wr_byte(0xcc);                    // skip ROM
    
     
    
      
      ow_wr_byte(0xBE);                     // start read
    
      #ifdef DEBUG 
      Serial.print("  Data = ");
      #endif
      for (int i=0; i<9; i++)               // read answer: 9 Byte Scratch Pad
      {
        data[i] = ow_rd_byte();
        DoJob2();
        #ifdef DEBUG 
        Serial.print(data[i], HEX); Serial.print(" ");
        #endif
      }
      #ifdef DEBUG 
      Serial.println();
      #endif
      
      uint8_t crc=0;                        // calculate and compare crc
      crc=crc8(data,8);
      #ifdef DEBUG
      Serial.print("CRC "); Serial.println(crc, HEX);
      #endif
      if (crc != data[8]) {
        Serial.println("CRC-Error");
        delay(1000);
        return;
      }
    
    
      
      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
    
      
      if (type_s[dev]) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } 
      else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        // default is 12 bit resolution, 600 ms conversion time
      }
      
      celsius[dev] = (float)raw / 16.0;
      celsius[dev] += Offset;
    
      // Filter
    
      if (!Out) {
        if (cel[dev]==0) {
          cel[dev]=celsius[dev];
          if (dev==(devices-1)) Out=true;
        }
      }
      
      cel[dev]=Alpha*cel[dev] + (1-Alpha)*celsius[dev];
      
      //fahrenheit = celsius * 1.8 + 32.0;
      #ifdef DEBUG 
      DoJob2();
      Serial.print(dev);
      DoJob2();
      Serial.print("  Temperature = ");
      DoJob2();
      Serial.print(celsius[dev],4); Serial.print(" ");
      DoJob2();
      Serial.print(cel[dev],4);      Serial.println(" Celsius ");
      DoJob2();
      #else
      
      if(Out) {
        for (int k=0; k<devices; k++) { 
          if(k==0) Serial.print((celsius[k])*1000-50,0); 
          else     Serial.print((celsius[k])*1000,0);
          DoJob2();
          Serial.print(" "); 
          DoJob2(); 
          if(k==0) Serial.print((cel[k])*1000-50,0); 
          else     Serial.print((cel[k])*1000,0);
          DoJob2();
          Serial.print(" "); 
          DoJob2();
          Serial.print( ( ((cel[0])*1000-50) +  ((cel[1])*1000) )/2 ,0); 
          DoJob2();
          Serial.print(" "); 
        }
        Serial.println();
        DoJob2();
          
      }
      #endif
    
    
      
    //  Serial.print(fahrenheit);
    //  Serial.println(" Fahrenheit");

  }
}




//----------------------------------------------- Multitasking: DoJob2 as state machine --------------

void DoJob2() {
static int entry;
static unsigned long Wait, Wait2;
static bool first=true;
static unsigned long LastWait;
  Job2Cnt++;                                  // about 10.000 in 0.6 seconds, => more than 16.000 per second
  if ((Job2Cnt % 50000)==0) MaxWait=0;        // Reset MaxWait from time to time

  if (first) {
    first=false;
    Wait=micros();
  }

  Wait2=micros()-Wait;
  if (Wait2>MaxWait) MaxWait=Wait2;           // MaxWait: longest time between two DoJob2-calls - about 550 µs
  
  // do multitasking
  switch (entry) {
    case 0: entry=1;
            Part0++;                          // about 2500 per 0.6 seconds, => more than 4000 per second !
            Wait=micros();
            return;
    case 1: entry=2;
            Part1++;                          // about 2500 per 0.6 seconds
            Wait=micros();
            return;
    case 2: entry=3;
            Part2++;                          // about 2500 per 0.6 seconds
            Wait=micros();
            return;
    case 3: entry=0;
            Wait=micros();
            if ((Wait-LastWait)>=1000) {
              Part3++;                        // every millisecond
              LastWait=Wait;
            }          
            return;            
  }
  
  
}
  


