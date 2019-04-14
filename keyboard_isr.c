// Part of the file provided by Terasic
#include <stdio.h>
#include "system.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

static QueueHandle_t Q_keydata;
double threshold_freq = 50.0;

int keyboard_isr(void* ps2_device, alt_u32 id)
{
  if(ps2_device == NULL){
    printf("can't find PS/2 device\n");
    return 1;
  }

  alt_up_ps2_clear_fifo (ps2_device) ;
  
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;  
  while(1)
  {
      // blocking function call      
      status = decode_scancode (ps2_device, &decode_mode , &key , &ascii) ;
      if ( status == 0 ) //success
      {
        // print out the result
        switch ( decode_mode )
        {
          case KB_ASCII_MAKE_CODE :
            printf ( "ASCII   : %x\n", key ) ;
            break ;
          case KB_BINARY_MAKE_CODE :
            printf ( "MAKE CODE : %x\n", key ) ;             
            break ;
          default :
            printf ( "DEFAULT   : %x\n", key ) ;
            break ;
        }
        IOWR(SEVEN_SEG_BASE, 0 ,key);
        xQueueSendtoBackFromISR(Q_keydata, &key, pdFALSE);
      }
  }  
  return 0;
}  

int update_frequency()
{
  char key;
  int pressed_key = 0;
  while(1)
  {
    xQueueReceive(Q_keydata, &key, pdFALSE);
    if (pressed_key == 1)
    {
      pressed_key == 0;
    }
    else
    {
      pressed_key == 1;
    }
    threshold_freq = atoi(key);
  }
}

int main()
{
  // creating queue
  Q_keydata = xQueueCreate(5, sizeof(char));

  // setting up device
  alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

  // registering ISR
  alt_irq_register(PS2_IRQ, ps2_device, keyboard_isr);
  //alt_up_ps2_enable_read_interrupt(ps2_device);
  IOWR_8DIRECT(PS2_BASE,4,1);
  while(1);
  return 0;
}
