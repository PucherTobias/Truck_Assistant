
#ifndef Fuzzy_F10
#define Fuzzy_F10 Fuzzy_F10

#include "fuzzy_enums.h"
#include "NumType_F10.h"
#include <assert.h>

typedef unsigned char NumOfVal_t;
typedef unsigned char Bool_t;

typedef struct{
  NumOfVal_t                n;
  const NumTypeF10Point_t* p;
}FuzzySetF10_t;


typedef struct{
  NumOfVal_t            n;
  const FuzzySetF10_t*  fs;
  NumTypeF10_t          defaultvalue;
  char                  defaultactive;
}LinguisticOutputVariableF10_t;

typedef struct{
  NumOfVal_t n;
  const FuzzySetF10_t* fs;
}LinguisticInputVariableF10_t;

typedef struct{
  NumOfVal_t nI;
  NumOfVal_t nO;
  NumOfVal_t nR;
  const LinguisticInputVariableF10_t* iL;
  const LinguisticOutputVariableF10_t* oL;
  const signed char* pre;   /* a negative value (e.g.: pre[3*nI+1]<0) indicates a negotiated set!!*/
  const char* con;          /* negative values doesn't occur here */
  const NumTypeF10_t* w;
  enum Inference_t inf;
  enum Defuzzy_t method;
  unsigned char steps;
  enum FuzzyAnd_t AndOp;
  enum FuzzyOr_t OrOp;
}FuzzyControllerF10_t;

 typedef struct{
    Bool_t **ihit; /* input hit [LingVar][Set] */
    Bool_t **ohit;
    NumTypeF10_t **imv;
    NumTypeF10_t **omv;
    NumTypeF10_t *res;
 }FCMemF10_t;


void FCF10_free(const FuzzyControllerF10_t *fc, FCMemF10_t *v);

void FCF10_calc(const FuzzyControllerF10_t *fc, FCMemF10_t *v, NumTypeF10_t *e, NumTypeF10_t *a);

void FCF10_init(const FuzzyControllerF10_t *fc, FCMemF10_t *v);

#endif

