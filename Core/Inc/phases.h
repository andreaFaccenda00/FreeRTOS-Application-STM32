// phases.h
#ifndef PHASES_H
#define PHASES_H

typedef enum {
  PHASE_1 = 1,  // sem1+3 VERDI, sem2 ROSSO
  PHASE_2,      // sem1+3 GIALLO, sem2 ROSSO
  PHASE_3,      // sem1+3 ROSSI, sem2 VERDI
  PHASE_4,      // sem1+3 ROSSI, sem2 GIALLO
  PHASE_5       // all-red + pedonale
} Phase_t;

// durate in ms
#define T_GREEN       4000U
#define T_YELLOW      1500U
#define T_PED_TOTAL   5000U
#define T_PED_BLINK   1000U
#define T_PED_INT     250U

#endif // PHASES_H
