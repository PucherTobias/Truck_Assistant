/***************************************************************************
  This file was created by the program FALCO which is a module 
  of WinFACT (Windows Fuzzy And Control Tools).

  Instead of editing this file, modify FALCOs Source-File
  with FLOP and generate it once again.

  Generated by : Florian
            on : DESKTOP-N6QTDVR
  FALCO Version: 8.0.0.9
  Source-File  : C:\Users\Florian\Documents\SCHULE\Diplomarbeit\Winfact-Geschwindigkeitsregelung\FuzzyV1.fuz from:30.12.2019 14:32:04
  Timestamp    : 30.12.2019 14:34:06

  Description of the fuzzy controller implemented in this code:
  =============================================================

    Inputs: 2
     e_winkel
     e_v

    Outputs: 2
     u_winkel
     u_v

    Rules: 10

    Inferencemechanism: Max-Min
    Method of defuzzyfication: center of gravity (Intergration via 60 steps)
    Number format: 4 Byte IEEE float
***************************************************************************/

#include "FuzzyV1_F4.h"

/*############################################################################*/
/*                        Sets of all input variables                         */
/*############################################################################*/

static const NumTypeF4Point_t FuzzyV1_F4_e_winkel_0_NegBig[3]={
   {-30, 0},
   {-30, 1},
   {-15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_winkel_1_NegSmall[3]={
   {-30, 0},
   {-15, 1},
   {0, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_winkel_2_Zero[3]={
   {-15, 0},
   {0, 1},
   {15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_winkel_3_PosSmall[3]={
   {0, 0},
   {15, 1},
   {30, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_winkel_4_PosBig[3]={
   {15, 0},
   {30, 1},
   {30, 0}};

static const NumTypeF4Point_t FuzzyV1_F4_e_v_0_NegBig[3]={
   {-30, 0},
   {-30, 1},
   {-15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_v_1_NegSmall[3]={
   {-30, 0},
   {-15, 1},
   {0, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_v_2_Zero[3]={
   {-15, 0},
   {0, 1},
   {15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_v_3_PosSmall[3]={
   {0, 0},
   {15, 1},
   {30, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_e_v_4_PosBig[3]={
   {15, 0},
   {30, 1},
   {30, 0}};

/*############################################################################*/
/*                        Sets of all output variables                        */
/*############################################################################*/

static const NumTypeF4Point_t FuzzyV1_F4_u_winkel_0_NegBig[3]={
   {-30, 0},
   {-30, 1},
   {-15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_winkel_1_NegSmall[3]={
   {-30, 0},
   {-15, 1},
   {0, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_winkel_2_Zero[3]={
   {-15, 0},
   {0, 1},
   {15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_winkel_3_PosSmall[3]={
   {0, 0},
   {15, 1},
   {30, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_winkel_4_PosBig[3]={
   {15, 0},
   {30, 1},
   {30, 0}};

static const NumTypeF4Point_t FuzzyV1_F4_u_v_0_NegBig[3]={
   {-30, 0},
   {-30, 1},
   {-15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_v_1_NegSmall[3]={
   {-30, 0},
   {-15, 1},
   {0, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_v_2_Zero[3]={
   {-15, 0},
   {0, 1},
   {15, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_v_3_PosSmall[3]={
   {0, 0},
   {15, 1},
   {30, 0}};
static const NumTypeF4Point_t FuzzyV1_F4_u_v_4_PosBig[3]={
   {15, 0},
   {30, 1},
   {30, 0}};


static const FuzzySetF4_t FuzzyV1_F4_e_winkel_0[5]={
   {3, FuzzyV1_F4_e_winkel_0_NegBig},
   {3, FuzzyV1_F4_e_winkel_1_NegSmall},
   {3, FuzzyV1_F4_e_winkel_2_Zero},
   {3, FuzzyV1_F4_e_winkel_3_PosSmall},
   {3, FuzzyV1_F4_e_winkel_4_PosBig}};

static const FuzzySetF4_t FuzzyV1_F4_e_v_1[5]={
   {3, FuzzyV1_F4_e_v_0_NegBig},
   {3, FuzzyV1_F4_e_v_1_NegSmall},
   {3, FuzzyV1_F4_e_v_2_Zero},
   {3, FuzzyV1_F4_e_v_3_PosSmall},
   {3, FuzzyV1_F4_e_v_4_PosBig}};
/*############################################################################*/
/*                              Input variables                               */
/*############################################################################*/

static const LinguisticInputVariableF4_t FuzzyV1_F4_Input[2]={
   {5, FuzzyV1_F4_e_winkel_0},
   {5, FuzzyV1_F4_e_v_1}};


static const FuzzySetF4_t FuzzyV1_F4_u_winkel_0[5]={
   {3, FuzzyV1_F4_u_winkel_0_NegBig},
   {3, FuzzyV1_F4_u_winkel_1_NegSmall},
   {3, FuzzyV1_F4_u_winkel_2_Zero},
   {3, FuzzyV1_F4_u_winkel_3_PosSmall},
   {3, FuzzyV1_F4_u_winkel_4_PosBig}};

static const FuzzySetF4_t FuzzyV1_F4_u_v_1[5]={
   {3, FuzzyV1_F4_u_v_0_NegBig},
   {3, FuzzyV1_F4_u_v_1_NegSmall},
   {3, FuzzyV1_F4_u_v_2_Zero},
   {3, FuzzyV1_F4_u_v_3_PosSmall},
   {3, FuzzyV1_F4_u_v_4_PosBig}};
/*############################################################################*/
/*                              Output variables                              */
/*############################################################################*/

static const LinguisticOutputVariableF4_t FuzzyV1_F4_Output[2]={
   {5, FuzzyV1_F4_u_winkel_0, 0, 0},
   {5, FuzzyV1_F4_u_v_1, 0, 0}};
/*############################################################################*/
/*                         Premises of the rule base                          */
/*############################################################################*/

static const char FuzzyV1_F4_RuleBase_pre[10*2]={
   3,0,
   2,0,
   1,0,
   4,0,
   5,0,
   0,3,
   0,2,
   0,1,
   0,4,
   0,5};

/*############################################################################*/
/*                        Conclusion of the rule base                         */
/*############################################################################*/

static const char FuzzyV1_F4_RuleBase_con[10*2]={
   3,0,
   4,0,
   5,0,
   2,0,
   1,0,
   0,3,
   0,2,
   0,1,
   0,4,
   0,5};

/*############################################################################*/
/*                           Weighting of the rules                           */
/*############################################################################*/

static const NumTypeF4_t FuzzyV1_F4_RuleBase_weight[10]={
   1,
   1,
   1,
   1,
   1,
   1,
   1,
   1,
   1,
   1};

/*############################################################################*/
/*                    All together in FuzzyControllerF4_t                     */
/*############################################################################*/
static const FuzzyControllerF4_t FuzzyV1_F4_FC={
   2,
   2,
   10,
   FuzzyV1_F4_Input,
   FuzzyV1_F4_Output,
   FuzzyV1_F4_RuleBase_pre,
   FuzzyV1_F4_RuleBase_con,
   FuzzyV1_F4_RuleBase_weight,
   MAX_MIN,
   DEFUZZY_COG,
   60,
   AND_MIN,
   OR_MAX};


/*############################################################################*/
/*                       Call this function to be sure                        */
/*                  to work with the correct number format.                   */
/*############################################################################*/


void FuzzyV1_F4_SetNumType(void)
{
}


/*############################################################################*/
/*                           Structure of pointers                            */
/*           This structure is allocated in the xx_init() function            */
/*        and is needed for the calculation in the xx_calc() function.        */
/*############################################################################*/

static FCMemF4_t FuzzyV1_F4_FCMem;


/*############################################################################*/
/*                          Initializiation function                          */
/*                  Call this function before the others !!                   */
/*############################################################################*/


void FuzzyV1_F4_init(void)
{
  FCF4_init(&FuzzyV1_F4_FC, &FuzzyV1_F4_FCMem);
}
/*############################################################################*/
/*                    Function representing the controller                    */
/*############################################################################*/


void FuzzyV1_F4_calc(
       const NumTypeF4_t i0,
       const NumTypeF4_t i1,
       NumTypeF4_t *o0,
       NumTypeF4_t *o1)
{
  NumTypeF4_t ai[2];
  NumTypeF4_t ao[2];
  ai[0]=i0;
  ai[1]=i1;
  FCF4_calc(&FuzzyV1_F4_FC, &FuzzyV1_F4_FCMem, ai, ao);
  *o0=ao[0];
  *o1=ao[1];
}
/*############################################################################*/
/*                       Function to release the memory                       */
/*                     which was allocated by xx-init().                      */
/*############################################################################*/


void FuzzyV1_F4_free(void)
{
  FCF4_free(&FuzzyV1_F4_FC, &FuzzyV1_F4_FCMem);
}
