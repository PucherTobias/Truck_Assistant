

#include <stdlib.h>
#include "fuzzy_F4.h"


static int membership(const FuzzySetF4_t *s, NumTypeF4_t e, NumTypeF4_t *mv)
{
  const NumTypeF4Point_t* p=s->p;
  NumOfVal_t n=s->n;

  if ( opF4L(e, p[0].x) || opF4G(e, p[n-1].x) ) return 0;
  if (n==1){ /*singleton*/
    if (opF4E(e, p[0].x)){
      *mv=1;
      return 1;
    }	
  }
  else{       /*polygon*/
    int i;
    for (i=1; i<s->n; i++)
      if (opF4LE(e, s->p[i].x)){
    	if (opF4NE(p[i].x, p[i-1].x)){
	  *mv=opF4Add(
		    opF4Div(
			  opF4Mul(
				opF4Sub(p[i].y,  p[i-1].y),
				opF4Sub(e,  p[i-1].x)
				),
			  opF4Sub(p[i].x,  p[i-1].x)
			  ),
		    p[i-1].y
		    );
        }else {
          if (opF4G(p[i].y, p[i-1].y)) *mv=p[i].y;
	  else *mv=p[i-1].y;
        }
        return *mv>0;
      }
  }
  return 0;
}

static NumTypeF4_t FuzzyAnd(const FuzzyControllerF4_t *fc, const NumTypeF4_t mue1, const NumTypeF4_t mue2)
{
  NumTypeF4_t zw;
  switch (fc->AndOp){
  case AND_BOUNDED_DIFF:
    zw=opF4Sub(opF4Add(mue1,mue2),F4One);
    return opF4G(zw,0) ? zw : 0;
  case AND_ALGEBRAIC_SUM:
    return opF4Mul(mue1,mue2);
  default: /* AND_MIN*/
    return (opF4G(mue1,mue2))? mue2 : mue1;
  }
}


static NumTypeF4_t FuzzyInference(const FuzzyControllerF4_t *fc,  const NumTypeF4_t mue, const NumTypeF4_t muex)
{
  switch (fc->inf){ 
  case  MAX_PROD :
    return opF4Mul(muex, mue);
  default: /*MAX_MIN*/   
    return opF4G(muex, mue) ? mue : muex;
  }
}


static void AproxCenterOfGravity(const FuzzyControllerF4_t *fc,  FCMemF4_t *v)
{
  const FuzzySetF4_t *s;
  NumTypeF4_t    x;
  NumTypeF4_t    n, z;
  NumOfVal_t  i, j;
  NumTypeF4_t   xmin, xdiff;

  for (i=0; i<fc->nO; i++){
    n=0; z=0;
    s=&fc->oL[i].fs[0];             /*s points to first set now*/
    xmin=s->p[0].x;
    s=&fc->oL[i].fs[fc->oL[i].n-1]; /*s points to last set now*/
    xdiff=opF4Sub(s->p[s->n-1].x,xmin);
    for (j=0; j<fc->oL[i].n; j++){
      s=&fc->oL[i].fs[j];              /*s points to actual set now*/
      if (!v->ohit[i][j]) continue;
      switch (s->n){
      case 1 /*Singleton*/  : x=s->p[0].x; break;
      case 3 /*Triangular*/ : x=s->p[1].x; break;
      case 4 /*Trapezoid*/  : x=opF4Div(opF4Add(s->p[1].x, s->p[2].x), opF4Cast(2));
      }
      /* scaling */
      x=opF4Div(opF4Sub(x,xmin),xdiff);
      /* end scaling */
      z=opF4Add(z, opF4Mul(v->omv[i][j], x));
      n=opF4Add(n, v->omv[i][j]);
    }
    if (n) v->res[i]=opF4Add(opF4Mul(opF4Div(z, n),xdiff),xmin);
    else if (fc->oL[i].defaultactive)
      v->res[i]=fc->oL[i].defaultvalue;
  }
}


static void ModifiedCenterOfGravity(const FuzzyControllerF4_t *fc,  FCMemF4_t *v)
{
  const FuzzySetF4_t *s0, *sn, *sb, *se, *s;
  NumTypeF4_t      dx;
  NumTypeF4_t      x;
  NumTypeF4_t      mv, mv0;
  NumTypeF4_t      mvx;
  NumTypeF4_t     n, z;
  NumTypeF4_t     n0, z0, nn, zn;
  NumOfVal_t    i, k;
  NumTypeF4_t     xmin, xdiff;

  for (i=0; i<fc->nO; i++){
    n=0; z=0; z0=0; n0=0; zn=0; nn=0;
    sb=NULL;
    se=NULL;
    s0=&fc->oL[i].fs[0];
    sn=&fc->oL[i].fs[fc->oL[i].n-1];
    for(s=s0; s!=sn+1; s++)
      if (v->ohit[i][s-s0]){
        se=s;
        if (sb==NULL) sb=s;
      }
    if (sb==NULL){
      if (fc->oL[i].defaultactive)
       	v->res[i]=fc->oL[i].defaultvalue;
      continue;
    }
    dx=opF4Div(
	     opF4Sub(se->p[se->n-1].x, sb->p[0].x),
	     opF4Cast(fc->steps)
	     );
    xmin=sb->p[0].x;
    xdiff=opF4Sub(se->p[se->n-1].x,xmin);
    se++;   /*points behind the last active set*/
    for (k=0; k<=fc->steps; k++){
      x=opF4Add(
	      opF4Mul( opF4Cast(k),	dx),
	      sb->p[0].x
	      );
      mv=0;
      for (s=sb; s!=se; s++){
        if (!v->ohit[i][s-s0]) continue;
        if (! membership(s, x, &mvx)) continue;
	mv0=FuzzyInference(fc,v->omv[i][s-s0],mvx);
	if (opF4L(mv, mv0)) mv=mv0;
        if (s==s0){  /*expansion to the left of the first set */
          NumTypeF4_t x0;
	  x0=opF4Sub(
		       opF4Add(s->p[0].x, 
				 s->p[0].x), 
		       x);
          if (opF4NE(x0, x)){  /*to be sure that no point is taken twice*/
	    mv0=FuzzyInference(fc,v->omv[i][s-s0],mvx);
	    /*scaling*/
	    x0=opF4Div(opF4Sub(x0, xmin),xdiff); 
	    /*end scaling*/
            z0=opF4Add(
			 opF4Mul(x0, mv0),
			 z0
			 );
            n0=opF4Add(n0, mv0);
          }
        }
        if (s==sn){  /*expansion to the right of the last set*/
          NumTypeF4_t xn;
	  xn=opF4Sub(
		       opF4Add(s->p[s->n-1].x,
				 s->p[s->n-1].x),
		       x);
          if (x!=xn){  /*to be sure that no point is taken twice*/
	    mv0=FuzzyInference(fc,v->omv[i][s-s0],mvx);
	    /*scaling*/
	    xn=opF4Div(opF4Sub(xn, xmin),xdiff); 
	    /*end scaling*/
	    zn=opF4Add(
			 opF4Mul(xn, mv0),
			 zn
			 );
            nn=opF4Add(nn, mv0);
          }
        }
      }
      /*scaling*/
      x=opF4Div(opF4Sub(x, xmin),xdiff); 
      /*end scaling*/
      z=opF4Add(
	      z,
	      opF4Mul(x, mv)
	      );
      n=opF4Add(n, mv);
    }
    z=opF4Add(opF4Add(z, z0), zn);
    n=opF4Add(opF4Add(n, n0), nn);
    if (n) v->res[i]=opF4Add(opF4Mul(opF4Div(z, n),xdiff),xmin);
  }
}



static void CenterOfGravity(const FuzzyControllerF4_t *fc,  FCMemF4_t *v)
{
  const FuzzySetF4_t *sb, *se, *s, *s0;
  NumTypeF4_t      dx;
  NumTypeF4_t      x;
  NumTypeF4_t      xmin, xdiff;
  NumTypeF4_t      mv;
  NumTypeF4_t      mvx;
  NumTypeF4_t      n, z;
  NumOfVal_t    i, k;

  for (i=0; i<fc->nO; i++){
    n=0; z=0;
    sb=NULL;
    se=NULL;
    s0=&fc->oL[i].fs[0];
    for (k=0; k<fc->oL[i].n; k++){
      if ((sb==NULL) && (v->ohit[i][k])) {
        sb=&fc->oL[i].fs[k];
      }
      if (v->ohit[i][k]){
        se=(&fc->oL[i].fs[k]);
      }
    }
    if (sb==NULL){
      if (fc->oL[i].defaultactive)
       	v->res[i]=fc->oL[i].defaultvalue;
      continue;
    }
    dx=opF4Div(
	     opF4Sub(se->p[se->n-1].x, sb->p[0].x),
	     opF4Cast(fc->steps)
	     );
    xmin=sb->p[0].x;
    xdiff=opF4Sub(se->p[se->n-1].x,xmin);
    se++;   /*points behind the last active set*/
    for (k=0; k<=fc->steps; k++){
      x=opF4Add(
	      opF4Mul(
		    opF4Cast(k),
		    dx
		    ),
	      sb->p[0].x
	      );
      mv=0;
      for (s=sb; s!=se; s++){
        if (!v->ohit[i][s-s0]) continue;
        if (!membership(s, x, &mvx)) continue;
	mvx=FuzzyInference(fc,v->omv[i][s-s0],mvx);
	if (opF4L(mv,mvx)) mv=mvx;
      }
      /*scaling*/
      x=opF4Div(opF4Sub(x, xmin),xdiff); 
      /*end scaling*/
      z=opF4Add(
	      z,
	      opF4Mul(x, mv)
	      );
      n=opF4Add(n, mv);
    }
    if (n) v->res[i]=opF4Add(opF4Mul(opF4Div(z, n),xdiff),xmin);
  }
}


static void maxheight(const FuzzyControllerF4_t *fc,  FCMemF4_t *v)
{
  const FuzzySetF4_t *s, *s0;
  NumTypeF4_t      mv;
  NumOfVal_t    i, j;

  for (i=0; i<fc->nO; i++){
    mv=0;
    for (j=0; j<fc->oL[i].n; j++){
      s=&fc->oL[i].fs[j];              /*s points to actual set now*/
      if (!v->ohit[i][j]) continue;
      if (((fc->method == DEFUZZY_MHR) && opF4LE(mv,v->omv[i][j])) ||  /*most right set with highest membership value*/
           opF4L(mv,v->omv[i][j]) ) {             /*most left set with highest membership value*/
        s0=s;
        mv=v->omv[i][j];
      }
    }
    if ((s0==NULL) && (fc->oL[i].defaultactive))
      v->res[i]=fc->oL[i].defaultvalue;
    if (s0==NULL) continue;
    switch (s0->n){
    case 1 /*Singleton*/  : v->res[i]=s0->p[0].x; break;
    case 3 /*Triangular*/ : v->res[i]=s0->p[1].x; break;
    case 4 /*Trapezoid*/  : v->res[i]=opF4Div(opF4Add(s0->p[1].x,s0->p[2].x),opF4Cast(2));
    }
  }
}


static void firstlastofmax(const FuzzyControllerF4_t *fc,  FCMemF4_t *v)
{
  const FuzzySetF4_t *s0;
  NumOfVal_t       i, j, k;
	char             found;
  char                no;

  for (i=0; i<fc->nO; i++){
    s0=NULL;
    no=0;
		found=0;
    for (j=0; j<fc->oL[i].n; j++){
      if (!v->ohit[i][j]) continue;
      if (v->omv[i][j]>v->omv[i][no]) {
				no=j;
				found=1;
			}
    }
    if (fc->oL[i].defaultactive)
      v->res[i]=fc->oL[i].defaultvalue;
    if (!found) continue;
    s0=&fc->oL[i].fs[no];
    k = ( fc->method == DEFUZZY_LOM ) ? s0->n-1 : 1;
    if (opF4Sub(s0->p[k].y, s0->p[k-1].y))
      v->res[i]=opF4Add(
					opF4Div(
							opF4Mul(
									opF4Sub(v->omv[i][no], s0->p[k-1].y),
									opF4Sub(s0->p[k].x, s0->p[k-1].x)
									),
							opF4Sub(s0->p[k].y, 
									s0->p[k-1].y)
					),s0->p[k-1].x);
  }
}

void FCF4_init(const FuzzyControllerF4_t *fc, FCMemF4_t *v)
{   
		NumOfVal_t  i;
		NumOfVal_t  lingvar;
		int		      sets;

		for (lingvar=0; lingvar<fc->nO; lingvar++){ /*over all ling. variables which are outputs*/
				for (sets=0; sets<fc->oL[lingvar].n; sets++){ /*over all sets of these */
						/* it is not allowed to use singleton sets with COG or MCOD: */
						assert(fc->oL[lingvar].fs[sets].n == 1 &&  
								(fc->method!=DEFUZZY_COG) && (fc->method!=DEFUZZY_MCOG));
				}
		}

		v->ihit = (Bool_t **)calloc(sizeof(char*), fc->nI);
		v->ohit = (Bool_t **)calloc(sizeof(char*), fc->nO);
		v->imv  = (NumTypeF4_t**)calloc(sizeof(NumTypeF4_t*), fc->nI);
		v->omv  = (NumTypeF4_t**)calloc(sizeof(NumTypeF4_t*), fc->nO);
		v->res  = (NumTypeF4_t*)calloc(sizeof(NumTypeF4_t), fc->nO);

		for (i=0; i<fc->nI; i++){ /*over all ling. variables which are inputs*/
				v->ihit[i] = (Bool_t *)calloc(sizeof(char), fc->iL[i].n);
				v->imv[i]  = (NumTypeF4_t*)calloc(sizeof(NumTypeF4_t), fc->iL[i].n);
		}
		for (i=0; i<fc->nO; i++){ /*over all ling. variables which are outputs*/
				v->ohit[i] = (Bool_t *)calloc(sizeof(char), fc->oL[i].n);
				v->omv[i]  = (NumTypeF4_t*)calloc(sizeof(NumTypeF4_t), fc->oL[i].n);
		}
		return ;
}


void FCF4_calc(const FuzzyControllerF4_t *fc, FCMemF4_t *v, NumTypeF4_t *e, NumTypeF4_t *a)
{
  NumOfVal_t i,j;
  /* calculate membership values for all sets of all linguistic inputs*/
  for (i=0; i<fc->nI; i++) /*over all ling. variables which are inputs*/
    for (j=0; j<fc->iL[i].n; j++) /*over all set of this ling. variable*/
      v->ihit[i][j] = membership(&fc->iL[i].fs[j], e[i], & v->imv[i][j]);

  /*reset hits of outputs set of all linguistic outputs*/

  for (i=0; i<fc->nO; i++)
    for (j=0; j<fc->oL[i].n; j++)
      v->ohit[i][j] = 0;

  /*check the rules*/
  for (i=0; i<fc->nR; i++){ /*over all premises*/
    NumTypeF4_t mv;
    signed char   index;
    NumTypeF4_t res=0;
    char          hit=0;

    for (j=0; j<fc->nI; j++){ /*over all ling. variables which are inputs*/
      if (!fc->pre[i*fc->nI+j]) continue; /*no ruler for this ling. variable !*/
      index=fc->pre[i*fc->nI+j]-1;
      if (fc->pre[i*fc->nI+j]<0){             /* make sure that index to fuzzy set is positiv and*/
	/* membershipvalue is correct (negated sets) */
        index=-index;
        mv=opF4Sub(F4One,v->imv[j][index]);                   /*set was negated so correct membershipvalue */
        v->ihit[j][index]=opF4G(mv,0);                    /*and also correct the hit flag*/
      }
      else{
        mv=v->imv[j][index];                       /*membershipvalue is ok */
      }
      if (v->ihit[j][index]){                      /*fuzzy set was hit by the input value*/
        if (!hit) {        /*ruler not jet hit || membershipvalue lower*/
          res = mv;
          hit = 1;
        }else{
          res=FuzzyAnd(fc, res, mv);
        }
      }else {
        hit=0;
        break;                      /*one part of the premise does not fit*/
      }
    }
    if (hit){
      res=opF4Mul(res, fc->w[i]);   /*multiply weightness*/
      for (j=0; j<fc->nO; j++){
        if (!fc->con[i*fc->nO+j]) continue;   /*no conclusion set*/
        index=fc->con[i*fc->nO+j]-1;
        if ((!v->ohit[j][index])||opF4L(v->omv[j][index], res)){     /*set was not hit by another rule or result membershipvalue higher*/
          v->omv[j][index]=res;
          v->ohit[j][index]=1;
        }
      }
    }
  }
  /*Defuzzification:*/
  switch (fc->method){
  case DEFUZZY_COG  : CenterOfGravity(fc, v); break;
  case DEFUZZY_MCOG : ModifiedCenterOfGravity(fc, v);  break;
  case DEFUZZY_ACOG : AproxCenterOfGravity(fc, v); break;
  case DEFUZZY_MHR  :
  case DEFUZZY_MHL  : maxheight(fc, v); break;
  case DEFUZZY_LOM  :
  case DEFUZZY_FOM  : firstlastofmax(fc, v); break;
  }
  for(i=0; i<fc->nO; i++)
    a[i]=v->res[i];
}


void FCF4_free(const FuzzyControllerF4_t *fc, FCMemF4_t *v)
{
  NumOfVal_t i;
  for (i=0; i<fc->nI; i++){ /*over all ling. variables which are inputs*/
    free(v->ihit[i]);
    free(v->imv[i]);
  }
  free(v->ihit);
  free(v->imv);
  for (i=0; i<fc->nO; i++){ /*over all ling. variables which are outputs*/
    free(v->ohit[i]);
    free(v->omv[i]);
  }
  free(v->ohit);
  free(v->omv);
  free(v->res);
}

