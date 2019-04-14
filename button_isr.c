#include "system.h"                     // to use the symbolic names
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts

int maintenanceFlag = 0;

// first we write our interrupt function
void button_interrupts_function(void* context, alt_u32 id)
{
  // need to cast the context first before using it
  //int* temp = (int*) context;
  (//*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
  
  if (IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) == 4){
	  if (maintenanceFlag == 0)
    {
	    printf("Maintenance mode ON");
      maintenanceFlag == 1;
    }
	  else
    {
      printf("Maintenance mode OFF");
      maintenanceFlag == 0;
    }
  }
	  

  // clears the edge capture register
  //IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

int main(void)
{

  // clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

  // enable interrupts for all buttons
  IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);

  // register the ISR
  alt_irq_register(PUSH_BUTTON_IRQ, NULL , button_interrupts_function);

  // need this to keep the program alive
  while(1);
  return 0;
}

