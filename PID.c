
//#include <PID.h>
#include <18F452.h>
#device ADC=10
#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES NOBROWNOUT               //No brownout reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
//#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#use delay(crystal=20000000)
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,stream=PORT1)
float t=0;

// --- Variáveis Globais ---
float   error_meas,
         kp = 6,
         ki = 0.015,
         kd = 0.0,
         proportional,
         integral = 0,
         derivative, 
         PID,
         ideal = 50;

int      timer=0,
         measure, pid_max;
         
int      lastMeasure=0;
int32 valor = 0;
int valor_anal; 
//interrupçao de timer
#int_TIMER1
void  TIMER1_isr(void) 
{
   timer += 1;
}

//interrupçao de timer externa 
#int_EXT
void  EXT_isr(void) 
{
   //fprintf (PORT1, "ponto zero\n") ;
   output_low (pin_b4) ;
   //delay_us (4000) ;
   delay_us (valor) ;
   output_high (pin_b4) ;
   clear_interrupt (int_EXT) ;
}

void serial()
{
   fprintf (PORT1, "--------\nTemp = %2u Graus\n Erro = %2f \nProp = %2f \nIntg = %2f \nPID= %2f\ntempo = %8f \n Valor = %2Lu\n --------\n", measure, error_meas, proportional, integral, PID, t, valor);
   // fprintf (PORT1, "Erro = % 2f \n", error_meas);
   //fprintf (PORT1, "Proporcional = % 2f \n", proportional);
   //fprintf (PORT1, "Integrativo = % 2f \n", integral);
   //fprintf (PORT1, "Derivativo = % 2f \n", derivative);
   //fprintf (PORT1, "tempo = % 8f \n", t);
   //fprintf (PORT1, "--------\n") ;
}

void main()
{
   setup_adc_ports (AN0) ;
   setup_adc (ADC_CLOCK_INTERNAL) ;
   set_adc_channel (0) ;
   setup_timer_1 (T1_INTERNAL|T1_DIV_BY_8); //104 ms overflow
   t = 0;
   enable_interrupts (int_TIMER1) ;
   enable_interrupts (int_EXT) ;
   ext_int_edge (INT_EXT, L_TO_H);
   enable_interrupts (GLOBAL) ;
   fprintf (PORT1, "Inicializando ProjetoPID\n") ;
   pid_max = ideal - 27;

   while (TRUE)
   {
      if (timer == 15)
      {
         valor_anal = read_adc (); // Lê o LM35
         measure = 5.0 * valor_anal * 100.0 / 1023.0; // Converte em graus celsius
         // Envia para a porta serial a Temperatura lida do LM35
         //--------------------------PID-------------------------------//
         error_meas = ideal - measure;
         proportional = error_meas * kp;
         integral += error_meas * ki;
         derivative = (lastMeasure - measure) * kd;
         lastMeasure = measure;
         PID = proportional + integral + derivative;
         t = (0.0083 + ( ( - 0.0083) * ( (PID) / (100))))  * 1000;

         if (t < 0)
         {
            t = t * - 1;
         }

         Serial () ;
         valor = t * 1000;

         if (valor > 8250)
         {
            valor = 8250;
         }

         timer = 0;
         //--------------------------PID-------------------------------//
      }
   }
}

