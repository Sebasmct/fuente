
#if !defined(Relay_on)
#include <EEPROM.h>
//#include "Wire.h"

//#include "Adafruit_INA219.h"

#define Relay_on LOW
#define Relay_off HIGH
#define DEBUG false
/*tiempos en  segundos */
// 6H  a segundos => H * 3600  => 6 * 3600= 21600 s
#define tiempo_reconexion 21600
//  42 minutos  a segundos  = 28800 s
#define tiempo_carga_adicional 4680
#endif


#define _error_vector 6
#define _error_stop 7
#define _error_corto 8
#define _vector_start 2
#define _vector_stop 1
#define _v_corto 4
#define pos_estado_vector 9
    int estados_vetor[pos_estado_vector] = {3, 2, 1, 6, 7, 8, 9, 5, 'A'};
    float corriente_vector[pos_estado_vector] = {20, 80, 120.0, 500.0, 1700.0, 4300};
    //int Rs232_vextor[pos_estado_vector] = {3, 2, 1, 6, 7, 8, 9, 5, 'A'};
  
